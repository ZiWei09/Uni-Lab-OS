#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
串口设备诊断工具
用于排查CH340串口转换器的连接和通信问题
"""

import serial
import time
import sys
from pathlib import Path

def test_serial_basic(port, baudrate=115200):
    """测试基本串口连接"""
    print(f"\n=== 基本串口测试: {port} ===")
    
    try:
        # 尝试打开串口
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False
        )
        
        print(f"✅ 串口打开成功")
        print(f"   端口: {ser.port}")
        print(f"   波特率: {ser.baudrate}")
        print(f"   超时: {ser.timeout}s")
        
        # 检查串口状态
        print(f"   DSR: {ser.dsr}")
        print(f"   CTS: {ser.cts}")
        print(f"   RI: {ser.ri}")
        print(f"   CD: {ser.cd}")
        
        # 清空缓冲区
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        # 测试基本读写
        print("📤 发送测试数据...")
        test_data = b'\x01\x03\x00\x00\x00\x01\x84\x0A'  # 简单的读取命令
        ser.write(test_data)
        print(f"   发送: {' '.join([f'{b:02X}' for b in test_data])}")
        
        # 等待响应
        time.sleep(0.5)
        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting)
            print(f"📥 接收到响应: {' '.join([f'{b:02X}' for b in response])}")
        else:
            print("⚠️  未收到响应")
        
        ser.close()
        print("✅ 串口测试完成")
        return True
        
    except serial.SerialException as e:
        print(f"❌ 串口错误: {e}")
        return False
    except Exception as e:
        print(f"❌ 其他错误: {e}")
        return False

def test_modbus_communication(port, baudrate=115200):
    """测试Modbus通信"""
    print(f"\n=== Modbus通信测试: {port} ===")
    
    try:
        ser = serial.Serial(port=port, baudrate=baudrate, timeout=2)
        
        # 测试不同的设备地址
        addresses = [0x01, 0x02, 0x03]  # 尝试多个设备地址
        
        for addr in addresses:
            print(f"\n🔍 测试设备地址: 0x{addr:02X}")
            
            # 构造读取命令 (功能码03：读保持寄存器)
            cmd = bytearray([
                addr,        # 设备地址
                0x03,        # 功能码：读保持寄存器
                0x00, 0x00,  # 起始地址
                0x00, 0x01   # 读取数量
            ])
            
            # 计算CRC
            crc = calculate_crc16(cmd)
            cmd.append(crc & 0xFF)
            cmd.append((crc >> 8) & 0xFF)
            
            print(f"   发送命令: {' '.join([f'{b:02X}' for b in cmd])}")
            
            # 清空缓冲区
            ser.reset_input_buffer()
            
            # 发送命令
            ser.write(cmd)
            
            # 等待响应
            time.sleep(0.2)
            
            if ser.in_waiting > 0:
                response = ser.read(ser.in_waiting)
                print(f"   收到响应: {' '.join([f'{b:02X}' for b in response])}")
                
                if len(response) >= 3 and response[0] == addr:
                    print(f"   ✅ 设备地址 0x{addr:02X} 响应正常")
                else:
                    print(f"   ⚠️  响应格式异常")
            else:
                print(f"   ❌ 设备地址 0x{addr:02X} 无响应")
        
        ser.close()
        
    except Exception as e:
        print(f"❌ Modbus测试失败: {e}")

def calculate_crc16(data):
    """计算Modbus CRC16"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc

def test_different_baudrates(port):
    """测试不同波特率"""
    print(f"\n=== 波特率测试: {port} ===")
    
    baudrates = [9600, 19200, 38400, 57600, 115200]
    
    for baudrate in baudrates:
        print(f"\n🔍 测试波特率: {baudrate}")
        try:
            ser = serial.Serial(port=port, baudrate=baudrate, timeout=1)
            
            # 发送简单命令
            cmd = b'\x01\x03\x00\x00\x00\x01\x84\x0A'
            ser.write(cmd)
            time.sleep(0.1)
            
            if ser.in_waiting > 0:
                response = ser.read(ser.in_waiting)
                print(f"   ✅ 波特率 {baudrate} 有响应: {' '.join([f'{b:02X}' for b in response])}")
            else:
                print(f"   ❌ 波特率 {baudrate} 无响应")
            
            ser.close()
            
        except Exception as e:
            print(f"   ❌ 波特率 {baudrate} 错误: {e}")

def check_system_info():
    """检查系统信息"""
    print("\n=== 系统信息检查 ===")
    
    # 检查内核模块
    try:
        import subprocess
        result = subprocess.run(['lsmod'], capture_output=True, text=True)
        if 'ch341' in result.stdout:
            print("✅ CH341驱动模块已加载")
        else:
            print("⚠️  CH341驱动模块未找到")
    except:
        pass
    
    # 检查设备节点
    import glob
    ch341_devices = glob.glob('/dev/ttyCH341USB*')
    if ch341_devices:
        print(f"✅ 找到CH341设备: {ch341_devices}")
    else:
        print("❌ 未找到CH341设备节点")
    
    # 检查权限
    try:
        import os
        import grp
        
        # 检查dialout组
        dialout_gid = grp.getgrnam('dialout').gr_gid
        user_groups = os.getgroups()
        
        if dialout_gid in user_groups:
            print("✅ 用户在dialout组中")
        else:
            print("❌ 用户不在dialout组中")
            
    except Exception as e:
        print(f"⚠️  权限检查失败: {e}")

def main():
    print("🔧 CH340串口设备诊断工具")
    print("=" * 50)
    
    # 系统信息检查
    check_system_info()
    
    # 检查设备列表
    port = '/dev/ttyCH341USB0'
    
    if not Path(port).exists():
        print(f"\n❌ 设备节点 {port} 不存在")
        
        # 查找其他可能的CH340设备
        import glob
        ch341_devices = glob.glob('/dev/ttyCH341USB*')
        if ch341_devices:
            print(f"发现其他CH341设备: {ch341_devices}")
            port = ch341_devices[0]
        else:
            print("未找到任何CH341设备")
            return
    
    # 基本串口测试
    if test_serial_basic(port):
        # Modbus通信测试
        test_modbus_communication(port)
        
        # 波特率测试
        test_different_baudrates(port)
    
    print("\n🏁 诊断完成")
    print("\n💡 解决建议:")
    print("1. 如果基本串口测试失败，检查设备连接和驱动")
    print("2. 如果Modbus测试失败，可能需要调整设备地址或波特率")
    print("3. 如果所有测试都失败，尝试重新插拔USB设备")
    print("4. 检查设备是否需要特定的初始化序列")

if __name__ == "__main__":
    main()