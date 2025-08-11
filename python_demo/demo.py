#!/usr/bin/env python
# -*- coding:utf-8 -*-
import serial
import struct
import platform
import serial.tools.list_ports
import math

def find_ttyUSB():
    print('IMU default serial port is /dev/ttyUSB0. If multiple serial devices are detected, modify the IMU port in the launch file.')
    ports = [port.device for port in serial.tools.list_ports.comports() if 'USB' in port.device]
    print('Connected {} serial devices: {}'.format(len(ports), ports))

def checkSum(list_data, check_data):
    return sum(list_data) & 0xff == check_data

def getSignInt16(value):
    return struct.unpack('h', struct.pack('H', value))[0]

def send_hex_data(serial_port, hex_string):
    """
    Send hexadecimal data string to the serial port.
    Example hex_string: "55 61 00 00 00 FF"
    """
    try:
        # Remove spaces and convert to bytes
        hex_bytes = bytes.fromhex(hex_string)
        serial_port.write(hex_bytes)
        # print(f"Sent: {hex_string}")
    except ValueError as e:
        print(f"Invalid hex string: {e}")

def handleSerialData(raw_data):
    global buff, key, acceleration, angularVelocity, angle_degree
    
    if python_version == '2':
        buff[key] = ord(raw_data)
    if python_version == '3':
        buff[key] = raw_data
    
    key += 1
    if buff[0] != 0x55:
        key = 0
        return
    if key < 20:
        return
    
    data_buff = list(buff.values())

    if buff[1] == 0x61:
            Ax = getSignInt16(data_buff[3] << 8 | data_buff[2]) / 32768.0 * 16
            Ay = getSignInt16(data_buff[5] << 8 | data_buff[4]) / 32768.0 * 16
            Az = getSignInt16(data_buff[7] << 8 | data_buff[6]) / 32768.0 * 16
            Gx = getSignInt16(data_buff[9] << 8 | data_buff[8]) / 32768.0 * 2000
            Gy = getSignInt16(data_buff[11] << 8 | data_buff[10]) / 32768.0 * 2000
            Gz = getSignInt16(data_buff[13] << 8 | data_buff[12]) / 32768.0 * 2000
            AngX = getSignInt16(data_buff[15] << 8 | data_buff[14]) / 32768.0 * 180
            AngY = getSignInt16(data_buff[17] << 8 | data_buff[16]) / 32768.0 * 180
            AngZ = getSignInt16(data_buff[19] << 8 | data_buff[18]) / 32768.0 * 180
            
            acceleration = [Ax, Ay, Az]
            angularVelocity = [Gx, Gy, Gz]
            angle_degree = [AngX, AngY, AngZ]
            
            print(
                f'''
                Acceleration (m/s²):
                    X-axis: {Ax:.3f}
                    Y-axis: {Ay:.3f}
                    Z-axis: {Az:.3f}
                
                Angular Velocity (deg/s):
                    X-axis: {Gx:.3f}
                    Y-axis: {Gy:.3f}
                    Z-axis: {Gz:.3f}
                
                Euler Angles (°):
                    X-axis: {AngX:.3f}
                    Y-axis: {AngY:.3f}
                    Z-axis: {AngZ:.3f}
                ''')
            send_hex_data(wt_imu, "FF AA 27 51 00")
    else:
        # print("///////////////////////////////////////////////////////////////////////////////")
        if buff[2] == 0x3A:
            Hx = getSignInt16(data_buff[5] << 8 | data_buff[4]) / 120
            Hy = getSignInt16(data_buff[7] << 8 | data_buff[6]) / 120
            Hz = getSignInt16(data_buff[9] << 8 | data_buff[8]) / 120
            magnetometer = [Hx, Hy, Hz]
            
            print(
                f'''
                Magnetometer (Gauss):
                    X-axis: {Hx:.3f}
                    Y-axis: {Hy:.3f}
                    Z-axis: {Hz:.3f}
                ''')
        elif buff[2] == 0x51:
            Q0 = getSignInt16(data_buff[5] << 8 | data_buff[4]) / 32768.0
            Q1 = getSignInt16(data_buff[7] << 8 | data_buff[6]) / 32768.0
            Q2 = getSignInt16(data_buff[9] << 8 | data_buff[8]) / 32768.0
            Q3 = getSignInt16(data_buff[11] << 8 | data_buff[10]) / 32768.0
            quaternion = [Q0, Q1, Q2, Q3]
            
            print(
                f'''
                Quaternion:
                    Q0: {Q0:.5f}
                    Q1: {Q1:.5f}
                    Q2: {Q2:.5f}
                    Q3: {Q3:.5f}
                ''')
        else:
            print("No parsing provided for " + str(buff[1]))
            print("Or data error")
            buff = {}
            key = 0
    
    buff = {}
    key = 0

key = 0
flag = 0
buff = {}
angularVelocity = [0, 0, 0]
acceleration = [0, 0, 0]
magnetometer = [0, 0, 0]
angle_degree = [0, 0, 0]
quaternion = [0, 0, 0, 0]
            
  


if __name__ == "__main__":
    python_version = platform.python_version()[0]
    
    find_ttyUSB()
    print("Operating System:", platform.system())
    port = "/dev/ttyUSB0" if platform.system().find("Linux") >= 0 else "COM4"
    
    baudrate = 115200
    
    try:
        wt_imu = serial.Serial(port=port, baudrate=baudrate, timeout=0.5)
        if wt_imu.isOpen():
            print("\033[32mSerial port opened successfully...\033[0m")
        else:
            wt_imu.open()
            print("\033[32mSerial port opened successfully...\033[0m")
    except Exception as e:
        print(e)
        print("\033[31mFailed to open serial port\033[0m")
        exit(0)
    else:
        while True:
            try:
                buff_count = wt_imu.inWaiting()
            except Exception as e:
                print("Exception:" + str(e))
                print("IMU lost connection, poor contact, or disconnected")
                exit(0)
            else:
                if buff_count > 0:
                    buff_data = wt_imu.read(buff_count)
                    for i in range(0, buff_count):
                        handleSerialData(buff_data[i])
