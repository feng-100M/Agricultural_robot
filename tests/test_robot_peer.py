"""离线验证发送端对 ACK 丢失、损坏和多次超时的处理。"""

import unittest

from robot_peer import send


class TestPort:
    """按顺序返回预设的串口应答，并保留每次写入的字节供断言比较。"""

    def __init__(self, replies):
        self.sent = []
        self.replies = iter(replies)

    def write(self, data):
        self.sent.append(data)
        return len(data)

    def read_until(self, marker, size):
        return next(self.replies)


class PeerTests(unittest.TestCase):
    def test_lost_or_corrupt_ack(self):
        """空回复模拟丢包，第二个回复的 CRC 有误，第三次收到有效 ACK 才递增序号。"""
        port = TestPort([b"", b"+00FFFF\n", b"+00E1F0\n"])
        self.assertEqual(send(port, 0, b"\x01\x00"), 1)
        self.assertEqual(port.sent, [b"!00020100D991\n"] * 3)

    def test_retry_exhausted(self):
        """三次读取均超时时应报告失败，发送次数不得超过约定上限。"""
        port = TestPort([b"", b"", b""])
        with self.assertRaises(TimeoutError):
            send(port, 0, b"\x01\x00")
        self.assertEqual(len(port.sent), 3)


if __name__ == "__main__":
    unittest.main()
