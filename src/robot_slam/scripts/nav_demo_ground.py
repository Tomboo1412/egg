#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
nav_demo_ground.py — 地面自主导航主控程序

比赛流程（状态机）:
  WAIT       : 等待语音唤醒信号 (/start_mission)
  OBSERVE    : 依次导航到 4 个任务信息图像观察点，停车拍照识别并播报
  GOTO_TASK  : 根据识别结果依次导航到 4 个黄色任务点并播报到达
  ENDPOINT   : 导航到终点 (格子 9) 并播报结束
  DONE       : 比赛结束

场地规格:
  3.6 m × 3.6 m, 9×9 格, 每格 0.4 m
  x 向右为正, y 向上为正
  格子 1 中心 → (0.2, 3.4)；格子编号从左上角横向排列 (1~81)

格子编号 → (x, y) 换算:
  row = (cell_id - 1) // 9 + 1
  col = (cell_id - 1) %  9 + 1
  x   = col * 0.4 - 0.2
  y   = 3.8 - row * 0.4

任务点映射 (task_id 1-9 → 格子编号):
  1→31  2→32  3→33
  4→40  5→41  6→42
  7→49  8→50  9→51
"""

import json
import rospy
import actionlib
import tf.transformations as tft

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, Quaternion
from std_msgs.msg import String
from std_srvs.srv import Trigger

# ──────────────────────────────────────────────────────────────────
# 场地常量
# ──────────────────────────────────────────────────────────────────
CELL_SIZE  = 0.4   # m / 格
GRID_COLS  = 9
GRID_ROWS  = 9
# 场地顶端 y 坐标 = GRID_ROWS 行格子顶端，单位 m
# 推导: 行1的上边界 = GRID_ROWS * CELL_SIZE = 3.6m，
#       行1格子中心 y = 3.6 - CELL_SIZE/2 = 3.4m
#       任意格子 y   = GRID_ROWS*CELL_SIZE - row*CELL_SIZE + CELL_SIZE/2
#                    = (GRID_ROWS + 0.5 - row) * CELL_SIZE
#                    = 3.8 - row * 0.4   (当 GRID_ROWS=9, CELL_SIZE=0.4)
_GRID_Y_OFFSET = (GRID_ROWS + 0.5) * CELL_SIZE   # = 3.8

# 4 个任务信息图像观察点格子编号（顺序固定）
OBSERVE_CELLS = [5, 45, 77, 37]

# task_id (1-9) → 格子编号
TASK_ID_TO_CELL = {
    1: 31, 2: 32, 3: 33,
    4: 40, 5: 41, 6: 42,
    7: 49, 8: 50, 9: 51,
}

ENDPOINT_CELL = 9   # 终点格子

# 识别服务名（与 identify_service.py 保持一致）
IDENTIFY_SERVICE = 'task_identify'

# 导航单个目标的超时时间（秒）
NAV_TIMEOUT = 120.0

# ──────────────────────────────────────────────────────────────────
# 坐标工具
# ──────────────────────────────────────────────────────────────────

def cell_id_to_xy(cell_id):
    """格子编号 (1-indexed) → 格子中心 (x, y) 坐标（米）"""
    row = (cell_id - 1) // GRID_COLS + 1
    col = (cell_id - 1) %  GRID_COLS + 1
    x = col  * CELL_SIZE - CELL_SIZE / 2.0
    y = _GRID_Y_OFFSET - row * CELL_SIZE
    return (x, y)


def yaw_to_quaternion(yaw_rad):
    """欧拉角 yaw → geometry_msgs/Quaternion"""
    q = tft.quaternion_from_euler(0.0, 0.0, yaw_rad)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


# ──────────────────────────────────────────────────────────────────
# 底盘控制工具
# ──────────────────────────────────────────────────────────────────

_vel_pub = None   # 由 GroundNavigator 初始化后赋值


def stop_robot(duration=0.5):
    """发布零速度，确保底盘完全停稳"""
    if _vel_pub is None:
        return
    zero = Twist()
    end = rospy.Time.now() + rospy.Duration(duration)
    rate = rospy.Rate(20)
    while rospy.Time.now() < end and not rospy.is_shutdown():
        _vel_pub.publish(zero)
        rate.sleep()


# ──────────────────────────────────────────────────────────────────
# 语音播报（优先调用 TTS 服务，失败则退化为 loginfo）
# ──────────────────────────────────────────────────────────────────

def speak(text):
    """语音播报。先停稳机器人再播报。"""
    stop_robot(0.3)
    rospy.loginfo(u'[播报] %s', text)
    try:
        from TTS_audio.srv import StringService
        rospy.wait_for_service('tts_service', timeout=2.0)
        tts_call = rospy.ServiceProxy('tts_service', StringService)
        tts_call(text)
    except Exception as exc:
        rospy.logwarn(u'TTS 服务不可用，仅记日志: %s', exc)


# ──────────────────────────────────────────────────────────────────
# 导航
# ──────────────────────────────────────────────────────────────────

def navigate_to(client, x, y, yaw=0.0, label=''):
    """
    向 move_base 发送单个目标并等待结果。
    返回 True 表示到达，False 表示超时或失败。
    """
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp    = rospy.Time.now()
    goal.target_pose.pose.position.x  = x
    goal.target_pose.pose.position.y  = y
    goal.target_pose.pose.orientation = yaw_to_quaternion(yaw)

    rospy.loginfo(u'导航目标 %s → (%.2f, %.2f, yaw=%.2f)', label, x, y, yaw)
    client.send_goal(goal)
    finished = client.wait_for_result(rospy.Duration(NAV_TIMEOUT))
    if not finished:
        rospy.logwarn(u'导航超时: %s', label)
        client.cancel_goal()
    return finished


# ──────────────────────────────────────────────────────────────────
# 识别
# ──────────────────────────────────────────────────────────────────

def capture_and_identify(obs_index, max_retry=3):
    """
    1. 触发摄像头拍照（设置 /detect 参数）
    2. 调用 task_identify 服务
    3. 解析 JSON {"task_id": N}，返回 1-9 或 None（失败）
    """
    # 触发拍照
    rospy.set_param('/detect', 1)
    rospy.sleep(1.5)   # 等待 identify_service 保存图像

    for attempt in range(1, max_retry + 1):
        try:
            rospy.wait_for_service(IDENTIFY_SERVICE, timeout=10.0)
            identify_srv = rospy.ServiceProxy(IDENTIFY_SERVICE, Trigger)
            resp = identify_srv()

            if resp.success:
                data = json.loads(resp.message)
                task_id = int(data.get('task_id', -1))
                if 1 <= task_id <= 9:
                    rospy.loginfo(u'第 %d 次识别成功: task_id=%d', obs_index + 1, task_id)
                    return task_id
                rospy.logwarn(u'识别结果超出合法范围: %s', data)
            else:
                rospy.logwarn(u'识别服务返回失败: %s', resp.message)

        except Exception as exc:
            rospy.logerr(u'识别调用异常 (第 %d/%d 次): %s', attempt, max_retry, exc)

        rospy.sleep(2.0)

    rospy.logerr(u'第 %d 个观察点识别失败（已重试 %d 次）', obs_index + 1, max_retry)
    return None


# ──────────────────────────────────────────────────────────────────
# 主状态机
# ──────────────────────────────────────────────────────────────────

class GroundNavigator:
    # 状态枚举
    STATE_WAIT      = 'WAIT'
    STATE_OBSERVE   = 'OBSERVE'
    STATE_GOTO_TASK = 'GOTO_TASK'
    STATE_ENDPOINT  = 'ENDPOINT'
    STATE_DONE      = 'DONE'

    def __init__(self):
        rospy.init_node('nav_demo_ground', anonymous=False)

        global _vel_pub
        _vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        # ── 读取 launch 参数（multi_goal.launch 通过 <param> 传入） ──
        #
        # goalListX / goalListY / goalListYaw 共 14 个点:
        #   索引 0-3  : 观察点 (格子 5, 45, 77, 37)
        #   索引 4-12 : 任务点 (格子 31-51，task_id 1-9 对应 index 4-12)
        #   索引 13   : 终点   (格子 9)
        raw_x   = rospy.get_param('~goalListX',   None)
        raw_y   = rospy.get_param('~goalListY',   None)
        raw_yaw = rospy.get_param('~goalListYaw', None)

        if raw_x is None or raw_y is None or raw_yaw is None:
            rospy.logfatal(
                'goalListX/Y/Yaw 参数未设置！请确认 multi_goal.launch 已正确传入参数。'
            )
            raise RuntimeError('Missing goalList params from launch file')

        self.goal_x   = [float(v.strip()) for v in str(raw_x).split(',')]
        self.goal_y   = [float(v.strip()) for v in str(raw_y).split(',')]
        self.goal_yaw = [float(v.strip()) for v in str(raw_yaw).split(',')]

        # 验证列表长度一致
        n = len(self.goal_x)
        if len(self.goal_y) != n or len(self.goal_yaw) != n:
            rospy.logerr('goalListX/Y/Yaw 长度不一致！请检查 multi_goal.launch')
        if n < 14:
            rospy.logwarn('goalList 只有 %d 个点，期望 14 个（4观察+9任务+1终点）', n)

        # 连接 move_base action server
        self._client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('等待 move_base action 服务器...')
        self._client.wait_for_server()
        rospy.loginfo('move_base 已连接')

        # 唤醒标志
        self._wakeup = False
        rospy.Subscriber('/start_mission', String, self._on_wakeup)

        self.state = self.STATE_WAIT
        self.task_sequence = []   # 识别得到的 task_id 列表（最多 4 个）

    # ── 语音唤醒回调 ──
    def _on_wakeup(self, msg):
        if not self._wakeup:
            self._wakeup = True
            rospy.loginfo(u'收到语音唤醒信号: %s', msg.data)

    # ── 等待唤醒 ──
    def _wait_for_wakeup(self):
        rospy.loginfo(u'[%s] 等待语音唤醒（/start_mission）...', self.STATE_WAIT)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self._wakeup:
            rate.sleep()

    # ── 导航到指定 goalList 索引的目标 ──
    def _nav_to_index(self, idx, label=''):
        if idx >= len(self.goal_x):
            rospy.logerr('目标索引 %d 超出 goalList 范围(%d)', idx, len(self.goal_x))
            return False
        return navigate_to(
            self._client,
            self.goal_x[idx],
            self.goal_y[idx],
            self.goal_yaw[idx],
            label=label,
        )

    # ── 主运行循环 ──
    def run(self):
        # ───────── WAIT ─────────
        self._wait_for_wakeup()
        self.state = self.STATE_OBSERVE
        stop_robot(0.5)
        speak(u'比赛开始')
        rospy.sleep(1.0)

        # ───────── OBSERVE ─────────
        rospy.loginfo(u'[%s] 开始识别任务信息图像', self.STATE_OBSERVE)
        for i, cell in enumerate(OBSERVE_CELLS):
            label = u'观察点%d(格子%d)' % (i + 1, cell)
            rospy.loginfo(u'前往 %s', label)
            self._nav_to_index(i, label=label)
            stop_robot(1.0)

            # 拍照识别
            task_id = capture_and_identify(i)
            if task_id is not None:
                self.task_sequence.append(task_id)
                goal_cell = TASK_ID_TO_CELL[task_id]
                speak(u'已识别第%d个任务信息，任务点编号%d，目标格子%d' % (
                    i + 1, task_id, goal_cell))
            else:
                rospy.logwarn(u'第 %d 个观察点识别失败，跳过', i + 1)
                speak(u'第%d个任务信息识别失败，跳过' % (i + 1,))

        rospy.loginfo(u'[OBSERVE 完成] 任务点序列: %s', self.task_sequence)

        # ───────── GOTO_TASK ─────────
        self.state = self.STATE_GOTO_TASK
        rospy.loginfo(u'[%s] 依次前往 %d 个任务点', self.STATE_GOTO_TASK,
                      len(self.task_sequence))

        for order, task_id in enumerate(self.task_sequence, start=1):
            if task_id not in TASK_ID_TO_CELL:
                rospy.logwarn(u'无效 task_id %d，跳过', task_id)
                continue
            goal_cell = TASK_ID_TO_CELL[task_id]
            # goalList 索引: 任务点从索引 4 开始，task_id 1 对应索引 4
            goal_idx = 4 + task_id - 1
            label = u'任务点%d(task_id=%d,格子%d)' % (order, task_id, goal_cell)
            rospy.loginfo(u'前往 %s (goalList[%d])', label, goal_idx)

            self._nav_to_index(goal_idx, label=label)
            stop_robot(1.0)
            speak(u'已到达第%d号任务点' % task_id)
            rospy.sleep(1.0)

        # ───────── ENDPOINT ─────────
        self.state = self.STATE_ENDPOINT
        end_idx = 13   # goalList 最后一个点 = 终点 (格子 9)
        rospy.loginfo(u'[%s] 前往终点 格子%d (goalList[%d])',
                      self.STATE_ENDPOINT, ENDPOINT_CELL, end_idx)
        self._nav_to_index(end_idx, label=u'终点(格子%d)' % ENDPOINT_CELL)
        stop_robot(1.5)
        speak(u'比赛结束')

        self.state = self.STATE_DONE
        rospy.loginfo(u'=== 比赛全部完成 ===')


# ──────────────────────────────────────────────────────────────────
# 入口
# ──────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    try:
        navigator = GroundNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        pass
