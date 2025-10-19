#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rospy
import serial
import struct
import platform
import serial.tools.list_ports
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import math

# Global initialization
buff = {}
key = 0

# Latest data storage
latest_accel = [0.0, 0.0, 0.0]
latest_gyro = [0.0, 0.0, 0.0]
latest_quat = [0.0, 0.0, 0.0, 0.0]
have_accel_gyro = False
have_quat = False

def find_ttyUSB():
    print('IMU default serial port is /dev/ttyUSB0. If multiple serial devices are detected, modify the IMU port in the launch file.')
    ports = [port.device for port in serial.tools.list_ports.comports() if 'USB' in port.device]
    print('Connected {} serial devices: {}'.format(len(ports), ports))

def getSignInt16(value):
    return struct.unpack('h', struct.pack('H', value))[0]

def send_hex_data(serial_port, hex_string):
    try:
        hex_bytes = bytes.fromhex(hex_string)
        serial_port.write(hex_bytes)
    except ValueError as e:
        print(f"Invalid hex string: {e}")

def handleSerialData(raw_data):
    global buff, key
    global latest_accel, latest_gyro, latest_quat
    global have_accel_gyro, have_quat

    buff[key] = raw_data
    key += 1

    if buff[0] != 0x55:
        key = 0
        return
    if key < 20:
        return

    data_buff = list(buff.values())

    # IMU acceleration + gyro packet
    if buff[1] == 0x61:
        latest_accel[0] = getSignInt16(data_buff[3] << 8 | data_buff[2]) / 32768.0 * 16 * 9.80665
        latest_accel[1] = getSignInt16(data_buff[5] << 8 | data_buff[4]) / 32768.0 * 16 * 9.80665
        latest_accel[2] = getSignInt16(data_buff[7] << 8 | data_buff[6]) / 32768.0 * 16 * 9.80665

        # Gyroscope: raw -> deg/s -> rad/s
        latest_gyro[0] = (getSignInt16(data_buff[9] << 8 | data_buff[8]) / 32768.0 * 2000) * math.pi / 180.0
        latest_gyro[1] = (getSignInt16(data_buff[11] << 8 | data_buff[10]) / 32768.0 * 2000) * math.pi / 180.0
        latest_gyro[2] = (getSignInt16(data_buff[13] << 8 | data_buff[12]) / 32768.0 * 2000) * math.pi / 180.0

        have_accel_gyro = True
        send_hex_data(wt_imu, "FF AA 27 51 00")

    # Quaternion packet
    elif buff[2] == 0x51:
        latest_quat[0] = getSignInt16(data_buff[5] << 8 | data_buff[4]) / 32768.0
        latest_quat[1] = getSignInt16(data_buff[7] << 8 | data_buff[6]) / 32768.0
        latest_quat[2] = getSignInt16(data_buff[9] << 8 | data_buff[8]) / 32768.0
        latest_quat[3] = getSignInt16(data_buff[11] << 8 | data_buff[10]) / 32768.0

        have_quat = True

    # Publish only if both data parts are ready
    if have_accel_gyro and have_quat:
        imu_msg = Imu()
        imu_msg.header = Header()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "trunk"

        imu_msg.linear_acceleration.x = latest_accel[0]
        imu_msg.linear_acceleration.y = latest_accel[1]
        imu_msg.linear_acceleration.z = latest_accel[2]

        imu_msg.angular_velocity.x = latest_gyro[0]
        imu_msg.angular_velocity.y = latest_gyro[1]
        imu_msg.angular_velocity.z = latest_gyro[2]

        imu_msg.orientation.x = latest_quat[1]
        imu_msg.orientation.y = latest_quat[2]
        imu_msg.orientation.z = latest_quat[3]
        imu_msg.orientation.w = latest_quat[0]

        # Orientation covariance
        imu_msg.orientation_covariance = [
            1.218e-05, 0, 0,
            0, 1.218e-05, 0,
            0, 0, 7.615e-05
        ]

        # Angular velocity covariance (rad/s)^2
        imu_msg.angular_velocity_covariance = [
            7.31e-07, 0, 0,
            0, 7.31e-07, 0,
            0, 0, 7.31e-07
        ]

        # Linear acceleration covariance (m/s^2)^2
        imu_msg.linear_acceleration_covariance = [
            7.38e-05, 0, 0,
            0, 7.38e-05, 0,
            0, 0, 7.38e-05
        ]

        imu_pub.publish(imu_msg)

        # Reset flags so we wait for new data
        have_accel_gyro = False
        have_quat = False

    buff.clear()
    key = 0

if __name__ == "__main__":
    rospy.init_node("imu_serial_node", anonymous=True)
    imu_pub = rospy.Publisher("/imu/data", Imu, queue_size=10)

    find_ttyUSB()
    port = "/dev/ttyUSB0" if platform.system().find("Linux") >= 0 else "COM3"
    baudrate = 115200

    try:
        wt_imu = serial.Serial(port=port, baudrate=baudrate, timeout=0.5)
        if wt_imu.isOpen():
            rospy.loginfo("Serial port opened successfully")
        else:
            wt_imu.open()
            rospy.loginfo("Serial port opened successfully")
    except Exception as e:
        rospy.logerr("Failed to open serial port: " + str(e))
        exit(0)

    while not rospy.is_shutdown():
        try:
            buff_count = wt_imu.inWaiting()
        except Exception as e:
            rospy.logerr("IMU lost connection: " + str(e))
            exit(0)
        if buff_count > 0:
            buff_data = wt_imu.read(buff_count)
            for i in range(buff_count):
                handleSerialData(buff_data[i])
