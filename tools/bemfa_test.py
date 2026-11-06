#!/usr/bin/env python3
"""
电脑端调试小工具 —— 不经过米家，直接验证链路。

使用前先设置环境变量（不要把自己的私钥写进代码里）：

    export BEMFA_UID=你的巴法云私钥          # cloud.bemfa.com 控制台右上角
    export BEMFA_TOPIC=你的主题名            # 例如 jimi006

用法：

    python tools/bemfa_test.py on                 # 让云端给 ESP32 发"开"（走 HTTP，不抢连接）
    python tools/bemfa_test.py off                # 让云端给 ESP32 发"关"
    python tools/bemfa_test.py status             # 读主题最新一条消息
    python tools/bemfa_test.py udp 192.168.1.50   # 绕过 ESP32，直接给投影仪发关机 UDP
    python tools/bemfa_test.py listen             # 用 MQTT 监听（注意：会顶掉 ESP32 的连接）

注意：巴法云的 clientID 必须完全等于私钥，同一个私钥同时只能有一个连接。
所以日常测试请用 on/off（HTTP 推送），不要用 listen。

只依赖标准库；listen 需要 pip install paho-mqtt
"""
import sys
import os
import json
import time
import socket
import urllib.request

UID = os.environ.get("BEMFA_UID", "")
TOPIC = os.environ.get("BEMFA_TOPIC", "jimi006")
HOST, PORT = "mqtt.bemfa.com", 9501


def _require_uid():
    if not UID:
        sys.exit("请先设置环境变量 BEMFA_UID（你的巴法云私钥）")


def _post(path: str, body: dict) -> dict:
    data = json.dumps(body).encode()
    req = urllib.request.Request(f"https://apis.bemfa.com/va/{path}", data=data,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=15) as r:
        return json.loads(r.read().decode())


def _get(path: str) -> dict:
    with urllib.request.urlopen(f"https://apis.bemfa.com/va/{path}", timeout=15) as r:
        return json.loads(r.read().decode())


def push(msg: str):
    _require_uid()
    res = _post("postJsonMsg", {"uid": UID, "topic": TOPIC, "type": 1, "msg": msg})
    ok = res.get("code") == 0
    print(f"[push] {TOPIC} <- {msg!r}  {'OK' if ok else res}")
    if not ok:
        print("       检查私钥/主题是否正确，或主题是否被删除")


def status():
    _require_uid()
    res = _get(f"getmsg?uid={UID}&topic={TOPIC}&type=1&num=1")
    data = res.get("data") or []
    if not data:
        print(f"[status] {TOPIC} 还没有消息")
        return
    print(f"[status] {TOPIC} 最新消息 = {data[0]['msg']!r}  ({data[0]['time']})")


def udp_off(ip: str):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.sendto(b"KEYPRESSES:30", (ip, 16735))
    time.sleep(0.12)
    s.sendto(b'{"action":20000,"controlCmd":{"delayTime":0,"mode":6,'
             b'"time":0,"type":0},"msgid":"2"}', (ip, 16750))
    s.close()
    print(f"[udp] 已向 {ip} 发送关机指令（16735 + 16750）")


def listen():
    _require_uid()
    import paho.mqtt.client as mqtt
    print("[warn] MQTT 连接会顶掉 ESP32，仅用于排查。Ctrl+C 退出")

    def on_connect(c, u, f, rc, props=None):
        print(f"[mqtt] connected rc={rc}, topic={TOPIC}")
        c.subscribe(TOPIC)

    c = mqtt.Client(client_id=UID, protocol=mqtt.MQTTv311,
                    callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
    c.on_connect = on_connect
    c.on_message = lambda c, u, m: print(
        f"[{time.strftime('%H:%M:%S')}] {m.topic} -> {m.payload.decode('utf-8', 'replace')}")
    c.connect(HOST, PORT, 30)
    try:
        c.loop_forever()
    except KeyboardInterrupt:
        print("\n[bye]")


if __name__ == "__main__":
    args = sys.argv[1:]
    if not args:
        print(__doc__)
    elif args[0] in ("on", "off"):
        push(args[0])
    elif args[0] == "status":
        status()
    elif args[0] == "udp" and len(args) > 1:
        udp_off(args[1])
    elif args[0] == "listen":
        listen()
    else:
        print(__doc__)
