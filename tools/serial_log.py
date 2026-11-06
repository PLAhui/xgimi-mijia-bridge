#!/usr/bin/env python3
"""把 ESP32 串口输出持续写到文件，方便事后排查。

用法：
    python tools/serial_log.py [秒数] [串口]

例：
    python tools/serial_log.py 600                     # 默认端口，监听 10 分钟
    python tools/serial_log.py 600 /dev/cu.usbserial-110
    python tools/serial_log.py 600 /dev/ttyUSB0        # Linux 常见端口

也可以用环境变量 SERIAL_PORT 指定端口。输出写到 /tmp/jimi_serial.log。
"""
import sys
import os
import time

import serial

SECONDS = int(sys.argv[1]) if len(sys.argv) > 1 else 300
PORT = (sys.argv[2] if len(sys.argv) > 2 else os.environ.get("SERIAL_PORT", "")) \
       or "/dev/cu.usbserial-110"
OUT = os.environ.get("SERIAL_LOG", "/tmp/jimi_serial.log")

s = serial.Serial(PORT, 115200, timeout=0.2)
with open(OUT, "a", encoding="utf-8") as f:
    f.write(f"\n===== 监听开始 {time.strftime('%H:%M:%S')} =====\n")
    f.flush()
    t0 = time.time()
    while time.time() - t0 < SECONDS:
        data = s.read(512)
        if data:
            f.write(data.decode("utf-8", "replace"))
            f.flush()
s.close()
with open(OUT, "a", encoding="utf-8") as f:
    f.write(f"\n===== 监听结束 {time.strftime('%H:%M:%S')} =====\n")
