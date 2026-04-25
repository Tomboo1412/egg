#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
API_KEY.py -- 大模型 API 密钥配置

密钥从环境变量读取，避免硬编码进源代码。
比赛/运行前在终端设置:
    export ARK_API_KEY="your_actual_api_key_here"

若环境变量未设置，YI_KEY 为空字符串，调用 API 时会报错并记录日志。
"""

import os

# 豆包 / 火山引擎 ARK 平台 API Key
YI_KEY = os.environ.get('ARK_API_KEY', '')
