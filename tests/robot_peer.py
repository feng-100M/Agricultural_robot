"""USART2 台架测试发送端。命令行入口以 115200 波特率发送心跳。
默认等待 ACK 50 ms，最多发送三次相同帧。首次请求为仅含心跳的 0 号帧。
发送端单独重启后，0 号帧仍须满足固件现有会话的序号规则。
心跳超时后保持停车。人工确认设备可安全启动、复位固件后，再启动发送端。
运行 python tests/robot_peer.py COM口 --count 100 可验证心跳收发；实物串口需要 pyserial。
离线运行 python -m unittest discover -s tests -p test_robot_peer.py。
"""

import argparse
import binascii
import time


def frame(sequence, payload):
    """将 0 至 255 的序号和至多 32 字节的 TLV 载荷编码为一行请求。
    CRC 覆盖二进制序号、长度及载荷，再转换为大写十六进制文本。
    """
    if len(payload) > 32:
        raise ValueError("TLV 载荷最多 32 字节")
    data = bytes((sequence, len(payload))) + payload
    crc = binascii.crc_hqx(data, 0xFFFF)
    return b"!" + data.hex().upper().encode("ascii") + f"{crc:04X}\n".encode("ascii")


def send(port, sequence, payload, attempts=3):
    """使用调用方持有的串口停等发送，port 须设置有限的读写超时。
    ACK 匹配才返回下一序号（255 后回到 0）。尝试 attempts 次均未确认则抛出 TimeoutError。
    """
    packet = frame(sequence, payload)
    crc = binascii.crc_hqx(bytes((sequence,)), 0xFFFF)
    expected = f"+{sequence:02X}{crc:04X}\n".encode("ascii")
    # 在重试之间复用原帧，接收端才能识别同一序号、同一内容的重复请求。
    for _ in range(attempts):
        if port.write(packet) == len(packet) and port.read_until(b"\n", 8) == expected:
            return (sequence + 1) & 0xFF
    raise TimeoutError(f"序号 {sequence:02X} 未收到有效确认")


if __name__ == "__main__":
    import serial

    parser = argparse.ArgumentParser(description="仅发送心跳，结束后进入固件心跳超时测试")
    parser.add_argument("port")
    parser.add_argument("--count", type=int, default=10)
    args = parser.parse_args()
    sequence = 0
    with serial.Serial(args.port, 115200, timeout=0.05, write_timeout=0.05) as connection:
        for _ in range(args.count):
            sequence = send(connection, sequence, b"\x01\x00")
            time.sleep(0.1)
