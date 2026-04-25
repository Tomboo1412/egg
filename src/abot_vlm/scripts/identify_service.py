#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
identify_service.py -- 任务信息图像识别服务

服务名: task_identify  (std_srvs/Trigger)
返回:
  success=True  -> message = JSON, e.g. '{"task_id": 3}'
  success=False -> message = JSON, e.g. '{"task_id": -1, "error": "..."}'

task_id 合法范围: 1-9，对应黄色任务点格子:
  1->31  2->32  3->33  4->40  5->41  6->42  7->49  8->50  9->51
"""

import json
import time
import base64
import sys
import os

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image as ROSImage
from std_srvs.srv import Trigger, TriggerResponse
from openai import OpenAI

# API Key -- 从环境变量读取，不要把 key 硬编码在代码中
# 比赛前在运行终端执行: export ARK_API_KEY="your_key_here"
YI_KEY = os.environ.get('ARK_API_KEY', '')

# 任务信息识别提示词
TASK_IDENTIFY_PROMPT = (
    '这是比赛场地围栏内侧贴的任务信息图像。'
    '图中标注了一个任务点编号，该编号是 1 到 9 中的某个整数。'
    '请识别图像中的任务点编号，最后一行只输出该数字（仅数字，无任何其他字符）；'
    '若无法识别，最后一行只输出"无"。'
)

VALID_TASK_IDS = {'1', '2', '3', '4', '5', '6', '7', '8', '9'}

# 图像保存路径: 优先从 ROS 参数服务器获取，允许 launch 文件覆盖
# 默认回退到原始路径以保持兼容性
_DEFAULT_SAVE_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'temp2', 'vl_now.jpg',
)
SAVE_PATH = os.environ.get('VLM_SAVE_PATH', _DEFAULT_SAVE_PATH)


# ------------------------------------------------------------------
# 图像格式转换
# ------------------------------------------------------------------

def imgmsg_to_cv2(img_msg):
    dtype = np.dtype('uint8')
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image = np.ndarray(
        shape=(img_msg.height, img_msg.width, 3),
        dtype=dtype,
        buffer=img_msg.data,
    )
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image = image.byteswap().newbyteorder()
    if img_msg.encoding == 'rgb8':
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    elif img_msg.encoding == 'mono8':
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    elif img_msg.encoding != 'bgr8':
        rospy.logerr('Unsupported encoding: %s', img_msg.encoding)
        return None
    return image


# ------------------------------------------------------------------
# 图像订阅回调 -- 按需保存图像
# ------------------------------------------------------------------

def on_image(img_msg):
    if rospy.get_param('/detect', 255) != 1:
        return
    img = imgmsg_to_cv2(img_msg)
    if img is None:
        return
    save_dir = os.path.dirname(SAVE_PATH)
    if save_dir and not os.path.exists(save_dir):
        os.makedirs(save_dir)
    cv2.imwrite(SAVE_PATH, img)
    rospy.loginfo('图像已保存至 %s', SAVE_PATH)
    rospy.set_param('/detect', 255)
    cv2.waitKey(1)


# ------------------------------------------------------------------
# 视觉大模型 API 调用
# ------------------------------------------------------------------

def call_vision_api(img_path=SAVE_PATH, max_retry=3):
    """
    调用豆包视觉大模型识别任务点编号。
    返回 '1'-'9' 或 None（失败）。
    """
    if not YI_KEY:
        rospy.logerr('ARK_API_KEY 环境变量未设置，无法调用大模型 API')
        return None

    client = OpenAI(
        api_key=YI_KEY,
        base_url='https://ark.cn-beijing.volces.com/api/v3',
    )

    try:
        with open(img_path, 'rb') as f:
            image_b64 = base64.b64encode(f.read()).decode('utf-8')
        image_url = 'data:image/jpeg;base64,' + image_b64
    except Exception as exc:
        rospy.logerr('读取图片失败: %s', exc)
        return None

    for attempt in range(1, max_retry + 1):
        try:
            response = client.chat.completions.create(
                model='doubao-1-5-vision-pro-32k-250115',
                messages=[{
                    'role': 'user',
                    'content': [
                        {'type': 'text', 'text': TASK_IDENTIFY_PROMPT},
                        {'type': 'image_url', 'image_url': {'url': image_url}},
                    ],
                }],
                timeout=30,
            )

            raw = response.choices[0].message.content.strip()
            rospy.loginfo('大模型原始返回:\n%s', raw)

            lines = raw.split('\n')
            last_line = lines[-1].strip() if lines else ''

            if last_line in VALID_TASK_IDS:
                return last_line

            # 兜底: 从末尾往前找第一个合法数字
            for ch in reversed(raw):
                if ch in VALID_TASK_IDS:
                    rospy.logwarn('兜底匹配到: %s', ch)
                    return ch

            rospy.logwarn('第 %d/%d 次: 未找到合法任务点编号，重试...', attempt, max_retry)

        except Exception as exc:
            rospy.logerr('API 调用失败 (第 %d/%d 次): %s', attempt, max_retry, exc)

        time.sleep(1.5)

    return None


# ------------------------------------------------------------------
# 服务处理函数
# ------------------------------------------------------------------

def handle_task_identify(req):
    result_str = call_vision_api()

    if result_str is not None and result_str in VALID_TASK_IDS:
        payload = json.dumps({'task_id': int(result_str)})
        rospy.loginfo('识别成功: %s', payload)
        return TriggerResponse(success=True, message=payload)

    error_msg = '识别失败或结果超出合法范围'
    rospy.logwarn('%s', error_msg)
    payload = json.dumps({'task_id': -1, 'error': error_msg})
    return TriggerResponse(success=False, message=payload)


# ------------------------------------------------------------------
# 主函数
# ------------------------------------------------------------------

def main():
    rospy.init_node('identify_node', anonymous=True)

    # 启动时检查 API Key
    if not YI_KEY:
        rospy.logwarn(
            'ARK_API_KEY 环境变量未设置！识别服务将无法调用大模型 API。'
            '请在启动前执行: export ARK_API_KEY="your_key_here"'
        )

    rospy.set_param('/detect', 255)
    rospy.Subscriber('/usb_cam/image_raw', ROSImage, on_image)
    rospy.Service('task_identify', Trigger, handle_task_identify)
    rospy.loginfo('任务信息识别服务已启动 (task_identify)，等待调用...')
    rospy.loginfo('图像保存路径: %s', SAVE_PATH)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
