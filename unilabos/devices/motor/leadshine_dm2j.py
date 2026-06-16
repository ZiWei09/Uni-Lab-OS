# -*- coding: utf-8 -*-
"""
Leadshine (雷赛) DM2J 系列驱控一体型步进驱动器
RS485 Modbus RTU 通信

支持型号: DM2J-RS522 / DM2J-RS542 / DM2J-RS556 / DM2J-RS570
"""

import time
import threading
import logging
from typing import Optional, Any

try:
    from pymodbus.client import ModbusSerialClient
except ImportError:
    ModbusSerialClient = None

try:
    from unilabos.device_comms.universal_driver import UniversalDriver
except ImportError:
    class UniversalDriver:
        def __init__(self, *args, **kwargs):
            self.logger = logging.getLogger(self.__class__.__name__)
        def execute_command_from_outer(self, command: Any): pass


# ============================================================
# Modbus 寄存器地址定义 (基于 DM2J 用户手册 V1.0)
# ============================================================

class Reg:
    """DM2J Modbus RTU 寄存器地址"""

    # 基本参数
    MICROSTEP = 0x0001          # Pr0.00 细分数 (默认 10000 P/R)
    MOTOR_DIRECTION = 0x0007    # Pr0.03 电机运行方向 (0=正, 1=反)
    FORCE_ENABLE = 0x000F       # Pr0.07 强制使能 (1=使能, 0=由IO控制)

    # 电流参数
    PEAK_CURRENT = 0x0191       # Pr5.00 峰值电流 (×0.1A)

    # 485 通讯参数
    BAUDRATE = 0x01BD           # Pr5.22 波特率
    SLAVE_ID = 0x01BF           # Pr5.23 站地址

    # JOG 参数 (RS485 通讯触发)
    JOG_SPEED = 0x01E1          # Pr6.00 JOG 速度 (RPM)
    JOG_ACC_DEC = 0x01E7        # Pr6.03 JOG 加减速时间 (ms)

    # 状态监控
    RUN_STATUS = 0x1003         # 运行状态 (Bit0=使能, Bit2=运行, Bit4=指令完成, Bit5=路径完成, Bit6=回零完成)

    # 辅助控制字
    CONTROL_WORD = 0x1801       # 控制字 (写入命令触发功能)

    # 报警
    ALARM = 0x2203              # 当前报警码

    # PR 控制
    PR_CONTROL = 0x6000         # Pr8.00 PR 控制参数
    PR_TRIGGER = 0x6002         # Pr8.02 触发寄存器

    # 回零参数
    HOME_MODE = 0x600A          # Pr8.10 回零模式
    HOME_SPEED_HIGH = 0x600F    # Pr8.15 回零高速 (RPM)
    HOME_SPEED_LOW = 0x6010     # Pr8.16 回零低速 (RPM)
    HOME_ACC = 0x6011           # Pr8.17 回零加速时间
    HOME_DEC = 0x6012           # Pr8.18 回零减速时间

    # 命令/电机位置 (只读)
    CMD_POSITION_H = 0x602A     # Pr8.42 命令位置高16位
    CMD_POSITION_L = 0x602B     # Pr8.43 命令位置低16位
    MOTOR_POSITION_H = 0x602C   # Pr8.44 电机位置高16位
    MOTOR_POSITION_L = 0x602D   # Pr8.45 电机位置低16位

    # PR 路径 0 参数 (立即触发用)
    PR0_MODE = 0x6200           # Pr9.00 路径0运动模式
    PR0_POS_H = 0x6201          # Pr9.01 路径0位置高16位
    PR0_POS_L = 0x6202          # Pr9.02 路径0位置低16位
    PR0_SPEED = 0x6203          # Pr9.03 路径0速度 (RPM)
    PR0_ACC = 0x6204            # Pr9.04 路径0加速时间 (ms/1000rpm)
    PR0_DEC = 0x6205            # Pr9.05 路径0减速时间 (ms/1000rpm)
    PR0_DELAY = 0x6206          # Pr9.06 路径0停顿时间
    PR0_TRIGGER = 0x6207        # Pr9.07 路径0触发 (映射到 Pr8.02, 写0x10立即触发)


# 控制字命令
class Cmd:
    """控制字 (写入 0x1801) 命令值"""
    RESET_ALARM = 0x1111        # 复位当前报警
    SAVE_PARAMS = 0x2211        # 保存所有参数到 EEPROM
    FACTORY_RESET = 0x2233      # 恢复出厂设置
    JOG_FORWARD = 0x4001        # 正向 JOG
    JOG_REVERSE = 0x4002        # 反向 JOG


# 触发寄存器命令 (写入 0x6002)
class Trigger:
    """触发寄存器命令值"""
    HOME = 0x0020               # 回零
    SET_ZERO = 0x0021           # 手动设零 (当前位置清零)
    ESTOP = 0x0040              # 急停

    @staticmethod
    def path(n: int) -> int:
        """触发路径 n (0-15): 写入 0x01P, P 为路径号 (即 0x0010+P)"""
        return 0x0010 | (n & 0x0F)


# 运行状态位定义
class StatusBit:
    ENABLED = 0x01      # Bit0: 使能
    FAULT = 0x02        # Bit1: 故障
    RUNNING = 0x04      # Bit2: 运行中
    CMD_DONE = 0x10     # Bit4: 指令完成
    PATH_DONE = 0x20    # Bit5: 路径完成
    HOME_DONE = 0x40    # Bit6: 回零完成


# PR 运动模式位定义
class PRMode:
    """PR 路径运动模式 (Pr9.00 各 bit 定义)"""
    TYPE_NONE = 0x0000      # 无动作
    TYPE_POSITION = 0x0001  # 位置定位
    TYPE_SPEED = 0x0002     # 速度运行
    TYPE_HOME = 0x0003      # 回零
    RELATIVE = 0x0040       # Bit6=1: 相对位置
    ABSOLUTE = 0x0000       # Bit6=0: 绝对位置


class LeadshineDM2JDriver(UniversalDriver):
    """
    雷赛 DM2J 系列驱控一体型步进驱动器

    支持功能:
    - 速度模式运行 (JOG)
    - 位置模式运行 (PR 立即触发，支持相对/绝对)
    - 位置读取和清零
    - 使能/禁用控制
    - 回零功能

    通信协议:
    - RS485 Modbus RTU
    - 功能码: 0x03(读), 0x06(写单个), 0x10(写多个)
    """

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        device_id: int = 1,
        timeout: float = 0.5,
        parity: str = "E",
        stopbits: int = 2,
        debug: bool = False
    ):
        """
        初始化 Leadshine DM2J 驱动

        Args:
            port: 串口设备路径
            baudrate: 波特率 (默认 115200, 支持 9600/19200/38400/115200)
            device_id: Modbus 从站地址 (1-31, 对应拨码开关设置)
            timeout: 通信超时时间(秒)
            parity: 校验位 ("E"=偶校验[出厂默认], "O"=奇校验, "N"=无校验)
            stopbits: 停止位 (2=出厂默认, 1=常用配置)
            debug: 是否启用调试输出
        """
        super().__init__()
        self.slave_id = device_id
        self.debug = debug
        self.lock = threading.RLock()
        self.status = "idle"
        self.position = 0
        self._jog_thread: Optional[threading.Thread] = None
        self._jog_stop = threading.Event()

        if ModbusSerialClient is None:
            self.logger.error("pymodbus 未安装，请运行: pip install pymodbus")
            self.client = None
            return

        try:
            self.client = ModbusSerialClient(
                port=port,
                baudrate=baudrate,
                bytesize=8,
                parity=parity,
                stopbits=stopbits,
                timeout=timeout,
            )
            if not self.client.connect():
                self.logger.error(f"无法连接到串口 {port}")
                self.client = None
                return

            self.logger.info(
                f"Leadshine DM2J connected: {port} "
                f"(Baud: {baudrate}, ID: {device_id})"
            )

            # 使能电机
            self.enable(True)

            # 启动背景轮询线程
            self._stop_event = threading.Event()
            self._polling_thread = threading.Thread(
                target=self._update_loop,
                name=f"DM2JPolling_{port}",
                daemon=True
            )
            self._polling_thread.start()

        except Exception as e:
            self.logger.error(f"Failed to open serial port {port}: {e}")
            self.client = None

    # ----------------------------------------------------------
    # 底层 Modbus 读写
    # ----------------------------------------------------------

    def _read_register(self, address: int, count: int = 1) -> Optional[list]:
        """读取保持寄存器"""
        if not self.client:
            return None
        with self.lock:
            result = self.client.read_holding_registers(
                address=address, count=count, slave=self.slave_id
            )
            if result.isError():
                if self.debug:
                    self.logger.warning(f"读取寄存器 0x{address:04X} 失败: {result}")
                return None
            return result.registers

    def _write_register(self, address: int, value: int) -> bool:
        """写入单个保持寄存器 (功能码 0x06)"""
        if not self.client:
            return False
        with self.lock:
            result = self.client.write_register(
                address=address, value=value, slave=self.slave_id
            )
            if result.isError():
                if self.debug:
                    self.logger.warning(f"写入寄存器 0x{address:04X}=0x{value:04X} 失败: {result}")
                return False
            return True

    def _write_registers(self, address: int, values: list) -> bool:
        """写入多个保持寄存器 (功能码 0x10)"""
        if not self.client:
            return False
        with self.lock:
            result = self.client.write_registers(
                address=address, values=values, slave=self.slave_id
            )
            if result.isError():
                if self.debug:
                    self.logger.warning(f"写入多个寄存器 0x{address:04X} 失败: {result}")
                return False
            return True

    def _read_32bit(self, addr_h: int) -> Optional[int]:
        """读取 32 位值 (高16位 + 低16位，连续两个寄存器)"""
        regs = self._read_register(addr_h, count=2)
        if regs is None or len(regs) < 2:
            return None
        return (regs[0] << 16) | regs[1]

    def _read_32bit_signed(self, addr_h: int) -> Optional[int]:
        """读取 32 位有符号值"""
        val = self._read_32bit(addr_h)
        if val is None:
            return None
        if val >= 0x80000000:
            val -= 0x100000000
        return val

    # ----------------------------------------------------------
    # 背景轮询
    # ----------------------------------------------------------

    def _update_loop(self):
        """背景循环读取电机位置和状态"""
        while not self._stop_event.is_set():
            try:
                self._poll_status()
            except Exception as e:
                if self.debug:
                    self.logger.error(f"Polling error: {e}")
            time.sleep(1.0)

    def _poll_status(self):
        """读取运行状态和位置"""
        # 读取运行状态 (JOG 运行时跳过状态更新，避免覆盖)
        if not self._jog_thread or not self._jog_thread.is_alive():
            regs = self._read_register(Reg.RUN_STATUS, count=1)
            if regs is not None:
                status_word = regs[0]
                if status_word & StatusBit.FAULT:
                    self.status = "fault"
                elif status_word & StatusBit.RUNNING:
                    self.status = "moving"
                elif status_word & StatusBit.ENABLED:
                    self.status = "idle"
                else:
                    self.status = "disabled"

        # 读取电机位置
        pos = self._read_32bit_signed(Reg.MOTOR_POSITION_H)
        if pos is not None:
            self.position = pos

    # ----------------------------------------------------------
    # 公共接口 (与 ZDT X42 保持一致)
    # ----------------------------------------------------------

    def enable(self, on: bool = True) -> bool:
        """
        使能/禁用电机

        Args:
            on: True=使能(锁轴), False=禁用(松轴)

        Returns:
            是否成功
        """
        return self._write_register(Reg.FORCE_ENABLE, 1 if on else 0)

    def move_speed(
        self,
        speed_rpm: int,
        direction: str = "CW",
        acceleration: int = 10
    ) -> bool:
        """
        速度模式运行 (JOG)

        通过 RS485 JOG 命令实现。后台线程每 40ms 重复发送 JOG 指令
        以保持连续运行 (手册要求 <50ms 间隔)。
        调用 stop() 停止运行。

        Args:
            speed_rpm: 转速 (RPM)
            direction: 方向 ("CW"=顺时针, "CCW"=逆时针)
            acceleration: 加减速时间 (ms)

        Returns:
            是否成功
        """
        # 先停止之前的 JOG
        self._stop_jog_thread()

        # 设置 JOG 速度和加减速
        self._write_register(Reg.JOG_SPEED, abs(speed_rpm))
        self._write_register(Reg.JOG_ACC_DEC, acceleration)

        jog_cmd = Cmd.JOG_FORWARD if direction.upper() in ["CW", "顺时针"] else Cmd.JOG_REVERSE

        # 发送第一次 JOG 命令
        if not self._write_register(Reg.CONTROL_WORD, jog_cmd):
            return False

        self.status = f"moving@{speed_rpm}rpm"

        # 启动后台线程持续发送 JOG (间隔 <50ms 才能连续运行)
        self._jog_stop.clear()
        self._jog_thread = threading.Thread(
            target=self._jog_keepalive,
            args=(jog_cmd,),
            daemon=True,
            name="DM2J_JOG"
        )
        self._jog_thread.start()
        return True

    def _jog_keepalive(self, jog_cmd: int):
        """后台线程：每 40ms 发送一次 JOG 命令维持连续运行"""
        while not self._jog_stop.is_set():
            self._write_register(Reg.CONTROL_WORD, jog_cmd)
            self._jog_stop.wait(0.04)  # 40ms 间隔

    def _stop_jog_thread(self):
        """停止 JOG 后台线程"""
        self._jog_stop.set()
        if self._jog_thread is not None and self._jog_thread.is_alive():
            self._jog_thread.join(timeout=0.2)
            self._jog_thread = None

    def move_position(
        self,
        pulses: int,
        speed_rpm: int,
        direction: str = "CW",
        acceleration: int = 10,
        absolute: bool = False
    ) -> bool:
        """
        位置模式运行 (PR 立即触发)

        Args:
            pulses: 脉冲数 (基于 10000 P/R)
            speed_rpm: 转速 (RPM)
            direction: 方向 ("CW"=顺时针, "CCW"=逆时针)
            acceleration: 加速时间 (ms/1000rpm)
            absolute: True=绝对位置, False=相对位置

        Returns:
            是否成功
        """
        # 先停止可能的 JOG
        self._stop_jog_thread()

        # 位置定位模式
        mode = PRMode.TYPE_POSITION
        if not absolute:
            mode |= PRMode.RELATIVE

        # 计算带符号位置
        target = int(pulses)
        if direction.upper() in ["CCW", "逆时针"]:
            target = -target

        # 转为 32 位无符号
        if target < 0:
            target_u32 = target + 0x100000000
        else:
            target_u32 = target
        pos_h = (target_u32 >> 16) & 0xFFFF
        pos_l = target_u32 & 0xFFFF

        values = [
            mode,           # Pr9.00 运动模式
            pos_h,          # Pr9.01 位置高16位
            pos_l,          # Pr9.02 位置低16位
            speed_rpm,      # Pr9.03 速度 (RPM)
            acceleration,   # Pr9.04 加速时间
            acceleration,   # Pr9.05 减速时间
            0,              # Pr9.06 停顿时间
            0x0010,         # Pr9.07 立即触发 (Pr9.07 映射到 Pr8.02, 写 0x10 触发 PR0)
        ]

        self.status = f"moving_to_{pulses}"
        return self._write_registers(Reg.PR0_MODE, values)

    def stop(self) -> bool:
        """
        停止电机 (急停)

        Returns:
            是否成功
        """
        self._stop_jog_thread()
        self.status = "idle"
        return self._write_register(Reg.PR_TRIGGER, Trigger.ESTOP)

    def is_moving(self) -> bool:
        """检查电机是否正在运动"""
        regs = self._read_register(Reg.RUN_STATUS, count=1)
        if regs is None:
            return False
        return bool(regs[0] & StatusBit.RUNNING)

    def wait_for_idle(self, timeout_s: float = 30.0, poll_interval: float = 0.1) -> bool:
        """
        等待电机运动完成

        Args:
            timeout_s: 最大等待时间(秒)
            poll_interval: 轮询间隔(秒)

        Returns:
            True=运动完成, False=超时
        """
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            regs = self._read_register(Reg.RUN_STATUS, count=1)
            if regs is not None:
                word = regs[0]
                if word & StatusBit.CMD_DONE and not (word & StatusBit.RUNNING):
                    self.status = "idle"
                    return True
            time.sleep(poll_interval)
        self.logger.warning(f"wait_for_idle 超时 ({timeout_s}s)")
        return False

    def rotate_quarter(self, speed_rpm: int = 60, direction: str = "CW") -> bool:
        """
        电机旋转 1/4 圈 (阻塞式)

        DM2J PR 模式下固定使用 10000 P/R，1/4 圈 = 2500 脉冲
        """
        pulses = 2500
        success = self.move_position(
            pulses=pulses, speed_rpm=speed_rpm,
            direction=direction, absolute=False
        )
        if success:
            estimated_time = 15.0 / max(1, speed_rpm)
            time.sleep(estimated_time + 0.5)
            self.status = "idle"
        return success

    def wait_time(self, duration_s: float) -> bool:
        """等待指定时间 (秒)"""
        self.logger.info(f"Waiting for {duration_s} seconds...")
        time.sleep(duration_s)
        return True

    def set_zero(self) -> bool:
        """
        将当前位置设为零点

        Returns:
            是否成功
        """
        return self._write_register(Reg.PR_TRIGGER, Trigger.SET_ZERO)

    def get_position(self) -> Optional[int]:
        """
        读取当前电机位置 (脉冲数)

        Returns:
            当前位置脉冲数，失败返回 None
        """
        pos = self._read_32bit_signed(Reg.MOTOR_POSITION_H)
        if pos is not None:
            self.position = pos
            if self.debug:
                self.logger.info(f"[Position] {self.position}")
            return self.position
        self.logger.warning("Failed to read position")
        return None

    # ----------------------------------------------------------
    # 扩展功能 (DM2J 特有)
    # ----------------------------------------------------------

    def home(self, direction: str = "CW", high_speed: int = 200, low_speed: int = 30) -> bool:
        """
        回零/回原点

        Args:
            direction: 回零方向 ("CW"=正向, "CCW"=反向)
            high_speed: 回零高速 (RPM)
            low_speed: 回零低速 (RPM)

        Returns:
            是否成功
        """
        # 配置回零参数
        mode_val = 0x04  # bit2=1: 原点回零
        if direction.upper() in ["CW", "正向"]:
            mode_val |= 0x01  # bit0=1: 正向

        self._write_register(Reg.HOME_MODE, mode_val)
        self._write_register(Reg.HOME_SPEED_HIGH, high_speed)
        self._write_register(Reg.HOME_SPEED_LOW, low_speed)

        # 触发回零
        self.status = "homing"
        return self._write_register(Reg.PR_TRIGGER, Trigger.HOME)

    def get_alarm(self) -> Optional[int]:
        """读取当前报警码"""
        regs = self._read_register(Reg.ALARM, count=1)
        if regs is not None:
            return regs[0]
        return None

    def clear_alarm(self) -> bool:
        """清除当前报警"""
        return self._write_register(Reg.CONTROL_WORD, Cmd.RESET_ALARM)

    def save_params(self) -> bool:
        """保存所有参数到 EEPROM"""
        return self._write_register(Reg.CONTROL_WORD, Cmd.SAVE_PARAMS)

    def set_current(self, current_a: float) -> bool:
        """
        设置峰值电流

        Args:
            current_a: 电流值 (A)，如 2.0 表示 2.0A

        Returns:
            是否成功
        """
        value = int(current_a * 10)
        return self._write_register(Reg.PEAK_CURRENT, value)

    def get_run_status(self) -> Optional[dict]:
        """
        读取详细运行状态

        Returns:
            包含各状态位的字典
        """
        regs = self._read_register(Reg.RUN_STATUS, count=1)
        if regs is None:
            return None
        word = regs[0]
        return {
            "enabled": bool(word & StatusBit.ENABLED),
            "fault": bool(word & StatusBit.FAULT),
            "running": bool(word & StatusBit.RUNNING),
            "cmd_done": bool(word & StatusBit.CMD_DONE),
            "path_done": bool(word & StatusBit.PATH_DONE),
            "home_done": bool(word & StatusBit.HOME_DONE),
        }

    def close(self):
        """关闭连接并停止所有线程"""
        self._stop_jog_thread()
        if hasattr(self, '_stop_event'):
            self._stop_event.set()
        if self.client:
            self.client.close()
            self.logger.info("Modbus connection closed")


# ============================================================
# 测试
# ============================================================

def test_motor():
    """基础功能测试"""
    logging.basicConfig(level=logging.INFO)

    print("=" * 60)
    print("Leadshine DM2J 步进电机驱动测试")
    print("=" * 60)

    driver = LeadshineDM2JDriver(
        port="/dev/tty.usbserial-0001",  # 根据实际修改
        baudrate=115200,
        device_id=1,
        debug=True
    )

    if not driver.client:
        print("串口连接失败")
        return

    try:
        # 测试 1: 读取位置
        print("\n[1] 读取当前位置")
        pos = driver.get_position()
        print(f"当前位置: {pos} 脉冲")

        # 测试 2: 读取运行状态
        print("\n[2] 读取运行状态")
        status = driver.get_run_status()
        print(f"状态: {status}")

        # 测试 3: 使能
        print("\n[3] 使能电机")
        driver.enable(True)
        time.sleep(0.3)

        # 测试 4: 相对位置运动
        print("\n[4] 相对位置运动 (1000脉冲)")
        driver.move_position(pulses=1000, speed_rpm=60, direction="CW")
        time.sleep(3)
        pos = driver.get_position()
        print(f"新位置: {pos}")

        # 测试 5: 速度运动
        print("\n[5] 速度模式 (30RPM, 3秒)")
        driver.move_speed(speed_rpm=30, direction="CW")
        time.sleep(3)
        driver.stop()
        pos = driver.get_position()
        print(f"停止后位置: {pos}")

        # 测试 6: 清零
        print("\n[6] 位置清零")
        driver.set_zero()
        time.sleep(0.3)
        pos = driver.get_position()
        print(f"清零后位置: {pos}")

        # 测试 7: 禁用
        print("\n[7] 禁用电机")
        driver.enable(False)

        print("\n" + "=" * 60)
        print("测试完成")
        print("=" * 60)

    except Exception as e:
        print(f"\n测试失败: {e}")
        import traceback
        traceback.print_exc()
    finally:
        driver.close()


if __name__ == "__main__":
    test_motor()
