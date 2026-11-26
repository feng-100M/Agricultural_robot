"""执行 Cortex-M7 固件中的任务、FreeRTOS 队列和异常入口。用法：
python tests/check_firmware_elf.py robot.elf
依赖 unicorn 2 和 pyelftools。使用含函数和静态对象符号的 Arm GCC ELF。
测试控制毫秒时钟、任务切换和外设输入，寄存器写入可检查，物理输出时延需上板测量。
"""

import struct
import sys
import unittest
from pathlib import Path

from robot_peer import frame
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_HOOK_CODE, UC_MODE_MCLASS, UC_MODE_THUMB
from unicorn.arm_const import (
    UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_SP,
    UC_CPU_ARM_CORTEX_M7,
)


TIM12 = 0x40001800
TIM15 = 0x40014000
GPIO_D_BSRR = 0x58020C18
PUMP_SET = 1 << 8
PUMP_RESET = 1 << 24
RCC_BASE = 0x58024400
DMA_STREAM = 0x40020010
DMA_BUFFER = 0x30000000
# DTCM 顶部留给模拟调用的工作区、栈和第 5 个以后的参数。
SCRATCH = 0x2001C000
STACK = 0x2001FF80
RETURN_ADDRESS = 0x0801FFF0


class Firmware:
    def __init__(self, path):
        self.cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        self.cpu.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M7)
        for address, size in ((0x08000000, 0x20000), (0x20000000, 0x20000),
                              (DMA_BUFFER, 0x1000), (0x40000000, 0x30000),
                              (0x58000000, 0x30000), (0xE000E000, 0x2000)):
            self.cpu.mem_map(address, size)
        with path.open("rb") as stream:
            elf = ELFFile(stream)
            self.symbols = {s.name: (s["st_value"], s["st_size"])
                            for s in elf.get_section_by_name(".symtab").iter_symbols()}
            for segment in elf.iter_segments():
                if segment["p_type"] == "PT_LOAD" and segment["p_filesz"]:
                    self.cpu.mem_write(segment["p_vaddr"], segment.data())
                if segment["p_type"] == "PT_LOAD" and 0x20000000 <= segment["p_vaddr"] < 0x20020000:
                    if segment["p_vaddr"] + segment["p_memsz"] > SCRATCH:
                        raise ValueError("固件 DTCM 段与测试工作区重叠")
        self.now_ms = 0
        self.write_word(self.address("htim12"), TIM12)
        self.write_word(self.address("htim15"), TIM15)
        self.stub("HAL_GetTick", lambda: self.now_ms)

    def address(self, name):
        return self.symbols[name][0]

    def write_word(self, address, value):
        self.cpu.mem_write(address, struct.pack("<I", value))

    def read_word(self, address):
        return int.from_bytes(self.cpu.mem_read(address, 4), "little")

    def stub(self, name, action):
        """截获函数入口，以 ARM 调用约定写回结果并返回调用者。"""
        def on_entry(cpu, address, size, data):
            result = action()
            if result is not None:
                cpu.reg_write(UC_ARM_REG_R0, result)
            cpu.reg_write(UC_ARM_REG_PC, cpu.reg_read(UC_ARM_REG_LR))

        entry = self.address(name) & ~1
        return self.cpu.hook_add(UC_HOOK_CODE, on_entry, begin=entry, end=entry)

    def call(self, name, *args, returns=True, limit=100000):
        self.cpu.reg_write(UC_ARM_REG_SP, STACK)
        for index, value in enumerate(args):
            if index < 4:
                self.cpu.reg_write(UC_ARM_REG_R0 + index, value)
            else:
                self.write_word(STACK + 4 * (index - 4), value)
        self.cpu.reg_write(UC_ARM_REG_LR, RETURN_ADDRESS | 1)
        self.cpu.emu_start(self.address(name) | 1, RETURN_ADDRESS, count=limit)
        if returns and self.cpu.reg_read(UC_ARM_REG_PC) != RETURN_ADDRESS:
            raise AssertionError(f"{name} 未在指令上限内返回")
        return self.cpu.reg_read(UC_ARM_REG_R0)

    def setup_tasks(self):
        for producer in ("control", "range"):
            prefix = f"g_robot_{producer}"
            slot, size = self.symbols[prefix + "_slot"]
            queue = self.call("xQueueGenericCreateStatic", 1, size, slot,
                              self.address(prefix + "_queue_storage"), 0)
            if not queue:
                raise AssertionError(f"{producer} 队列创建失败")
            self.write_word(self.address(prefix + "_queue"), queue)
        yaw_queue = self.call("xQueueGenericCreateStatic", 1, 4, SCRATCH, SCRATCH + 64, 0)
        self.write_word(self.address("YawDataHandle"), yaw_queue)

        self.pending = b""
        self.ack = lambda: 0
        self.refreshes = 0
        self.stub("robot_uart_start", lambda: 1)
        self.stub("robot_uart_read", self.read_input)
        self.stub("HAL_UART_Transmit", lambda: self.ack())
        self.stub("robot_watchdog_refresh", self.refresh)
        self.stub("ulTaskNotifyTake", self.wait_for_input)

    def refresh(self):
        self.refreshes += 1

    def read_input(self):
        count = min(len(self.pending), self.cpu.reg_read(UC_ARM_REG_R1))
        if count:
            self.cpu.mem_write(self.cpu.reg_read(UC_ARM_REG_R0), self.pending[:count])
        self.pending = self.pending[count:]
        return count

    def wait_for_input(self):
        self.waiting = True
        self.cpu.emu_stop()

    def publish(self, now_ms, packet=b""):
        self.waiting = False
        self.pending = packet
        self.now_ms = now_ms
        self.call("StartDefaultTask", 0, returns=False)
        if self.pending or not self.waiting:
            raise AssertionError("通信任务未完成本轮接收")

    def run_cycles(self, name, count):
        """在任务完成指定轮数、进入下一轮延时时暂停。"""
        delays = 0

        def delay():
            nonlocal delays
            delays += 1
            if delays == count + 1:
                self.cpu.emu_stop()

        hook = self.stub("vTaskDelay", delay)
        try:
            self.call(name, 0, returns=False)
        finally:
            self.cpu.hook_del(hook)
        if delays != count + 1:
            raise AssertionError(f"{name} 未完成 {count} 轮")

    def motor(self, now_ms):
        self.now_ms = now_ms
        self.run_cycles("StartMotor", 1)
        return self.read_word(GPIO_D_BSRR)


class FirmwareChecks(unittest.TestCase):
    def setUp(self):
        self.fw = Firmware(self.elf_path)

    def test_latest_control(self):
        fw = self.fw
        fw.setup_tasks()
        fw.publish(100, b"@d\n")
        fw.publish(120, b"@n\n")
        self.assertEqual(fw.motor(121), PUMP_RESET)
        fw.publish(125, b"@n\n")
        fw.publish(130, b"@d\n")
        self.assertEqual(fw.motor(131), PUMP_SET)

    def test_latest_range(self):
        fw = self.fw
        fw.setup_tasks()
        samples = iter(((10, 20, 30), (40, 50, 60)))

        def read_range():
            fw.cpu.mem_write(fw.cpu.reg_read(UC_ARM_REG_R0), struct.pack("<3H", *next(samples)))
            return 1

        fw.stub("robot_range_init", lambda: 1)
        fw.stub("robot_range_read", read_range)
        fw.run_cycles("StartTof_F", 2)
        queue = fw.read_word(fw.address("g_robot_range_queue"))
        output = SCRATCH + 256
        self.assertEqual(fw.call("xQueueReceive", queue, output, 0), 1)
        self.assertEqual(struct.unpack("<3H", fw.cpu.mem_read(output, 6)), (40, 50, 60))
        self.assertEqual(fw.call("xQueueReceive", queue, output, 0), 0)

    def test_legacy_deadline(self):
        fw = self.fw
        fw.setup_tasks()
        fw.publish(100, b"@d\n")
        fw.publish(590)
        self.assertEqual(fw.motor(599), PUMP_SET)
        self.assertEqual(fw.motor(600), PUMP_RESET)

    def test_heartbeat_during_blocked_ack(self):
        fw = self.fw
        fw.setup_tasks()
        fw.publish(100, b"!00020100D991\n")
        acks = []

        def acknowledge():
            acks.append(fw.now_ms)
            fw.now_ms = 600
            return 0

        fw.ack = acknowledge
        fw.publish(599, b"!00020100D991\n!01020100AF25\n")
        fw.publish(601, b"!01020100AF25\n")
        self.assertEqual(acks, [599])

    def test_heartbeat_stops_outputs_until_reset(self):
        fw = self.fw
        fw.setup_tasks()
        fw.publish(100, frame(0, bytes.fromhex("0100")))
        fw.publish(101, frame(1, bytes.fromhex("020101")))
        fw.publish(590)
        self.assertEqual(fw.motor(599), PUMP_SET)
        for timer in (TIM12, TIM15):
            fw.write_word(timer + 0x34, 30)
            fw.write_word(timer + 0x38, 80)
        self.assertEqual(fw.motor(600), PUMP_RESET)
        for timer in (TIM12, TIM15):
            self.assertEqual(fw.read_word(timer + 0x34), 0)
            self.assertEqual(fw.read_word(timer + 0x38), 0)
        fw.publish(601, frame(2, bytes.fromhex("0100")))
        self.assertEqual(fw.motor(602), PUMP_RESET)

    def test_lost_progress_stops_watchdog_refresh(self):
        fw = self.fw
        fw.setup_tasks()
        fw.publish(10, b"@d\n")
        self.assertEqual(fw.motor(259), PUMP_SET)
        self.assertEqual(fw.refreshes, 1)
        self.assertEqual(fw.motor(260), PUMP_RESET)
        self.assertEqual(fw.refreshes, 1)
        fw.publish(261, b"@d\n")
        self.assertEqual(fw.motor(262), PUMP_RESET)

    def test_fault_outputs(self):
        entries = ("Error_Handler", "NMI_Handler", "HardFault_Handler", "BusFault_Handler",
                   "MemManage_Handler", "UsageFault_Handler", "xQueuePeek")
        for entry in entries:
            with self.subTest(entry=entry):
                fw = self.fw
                for offset in (0xE0, 0xE8, 0xF0):
                    fw.write_word(RCC_BASE + offset, 0xFFFFFFFF)
                fw.write_word(GPIO_D_BSRR, PUMP_SET)
                for timer in (TIM12, TIM15):
                    fw.write_word(timer + 0x34, 30)
                    fw.write_word(timer + 0x38, 80)
                    fw.write_word(timer + 0x14, 0)
                # 空句柄触发内核自身的 configASSERT 路径。
                args = (0, SCRATCH, 0) if entry == "xQueuePeek" else ()
                fw.call(entry, *args, returns=False, limit=500)
                self.assertEqual(fw.read_word(GPIO_D_BSRR), PUMP_RESET)
                for timer in (TIM12, TIM15):
                    self.assertEqual(fw.read_word(timer + 0x34), 0)
                    self.assertEqual(fw.read_word(timer + 0x38), 0)
                    self.assertEqual(fw.read_word(timer + 0x14), 1)

    def test_fault_before_clocks_enabled(self):
        fw = self.fw
        for timer in (TIM12, TIM15):
            fw.write_word(timer + 0x34, 17)
        fw.call("Error_Handler", returns=False, limit=500)
        self.assertEqual(fw.read_word(GPIO_D_BSRR), 0)
        for timer in (TIM12, TIM15):
            self.assertEqual(fw.read_word(timer + 0x34), 17)
            self.assertEqual(fw.read_word(timer + 0x14), 0)

    def test_dma_wrap(self):
        fw = self.fw
        fw.write_word(fw.address("g_robot_uart_dma"), DMA_STREAM)
        fw.write_word(fw.address("g_robot_uart_task"), 1)
        fw.stub("vTaskNotifyGiveFromISR", lambda: None)
        sent = bytes((i * 7 + i // 256) % 256 for i in range(2048))
        received = bytearray()
        for offset in range(0, len(sent), 64):
            position = offset % 256
            fw.cpu.mem_write(DMA_BUFFER + position, sent[offset:offset + 64])
            fw.write_word(DMA_STREAM + 4, 256 - (position + 64) % 256)
            for _ in range(2):
                fw.call("HAL_UARTEx_RxEventCallback", fw.address("huart2"), position + 64)
            for _ in range(3):
                size = fw.call("robot_uart_read", SCRATCH, 64, SCRATCH + 64)
                self.assertEqual(fw.cpu.mem_read(SCRATCH + 64, 1), b"\0")
                received.extend(fw.cpu.mem_read(SCRATCH, size))
        self.assertEqual(received, sent)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    FirmwareChecks.elf_path = Path(sys.argv.pop(1))
    unittest.main()
