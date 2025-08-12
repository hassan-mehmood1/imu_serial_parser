#!/usr/bin/env python
# -*- coding:utf-8 -*-
import rospy
import serial
import struct
import platform
import serial.tools.list_ports
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Header, Float32
from tf.transformations import quaternion_from_euler
import math  # For quaternion conversion

# Global initialization of buff and key
buff = {}  # This is where the raw data will be stored
key = 0  # This keeps track of the data index

# Define new publishers for angles and quaternions
euler_roll_pub = rospy.Publisher("/imu/euler_roll", Float32, queue_size=10)  # Topic for roll angle
euler_pitch_pub = rospy.Publisher("/imu/euler_pitch", Float32, queue_size=10)  # Topic for pitch angle
euler_yaw_pub = rospy.Publisher("/imu/euler_yaw", Float32, queue_size=10)  # Topic for yaw angle
quaternion_pub = rospy.Publisher("/imu/quaternion", Imu, queue_size=10)  # Topic for Quaternion

def find_ttyUSB():
    print('IMU default serial port is /dev/ttyUSB0. If multiple serial devices are detected, modify the IMU port in the launch file.')
    ports = [port.device for port in serial.tools.list_ports.comports() if 'USB' in port.device]
    print('Connected {} serial devices: {}'.format(len(ports), ports))

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
    global buff, key, imu_pub, mag_pub, euler_roll_pub, euler_pitch_pub, euler_yaw_pub, quaternion_pub
    
    buff[key] = raw_data
    key += 1

    if buff[0] != 0x55:  # Start byte check
        key = 0
        return
    if key < 20:
        return

    data_buff = list(buff.values())
    imu_msg = Imu()
    imu_msg.header = Header()
    imu_msg.header.stamp = rospy.Time.now()
    imu_msg.header.frame_id = "base_imu"

    if buff[1] == 0x61:  # IMU data (acceleration, gyro, euler angles)
        # imu_msg = Imu()
        # imu_msg.header = Header()
        # imu_msg.header.stamp = rospy.Time.now()
        # imu_msg.header.frame_id = "base_imu"

        # Acceleration
        imu_msg.linear_acceleration.x = getSignInt16(data_buff[3] << 8 | data_buff[2]) / 32768.0 * 16
        imu_msg.linear_acceleration.y = getSignInt16(data_buff[5] << 8 | data_buff[4]) / 32768.0 * 16
        imu_msg.linear_acceleration.z = getSignInt16(data_buff[7] << 8 | data_buff[6]) / 32768.0 * 16

        # Angular Velocity
        imu_msg.angular_velocity.x = getSignInt16(data_buff[9] << 8 | data_buff[8]) / 32768.0 * 2000
        imu_msg.angular_velocity.y = getSignInt16(data_buff[11] << 8 | data_buff[10]) / 32768.0 * 2000
        imu_msg.angular_velocity.z = getSignInt16(data_buff[13] << 8 | data_buff[12]) / 32768.0 * 2000

        # Euler Angles (Degrees to Radians)
        roll = getSignInt16(data_buff[15] << 8 | data_buff[14]) / 32768.0 * 180
        pitch = getSignInt16(data_buff[17] << 8 | data_buff[16]) / 32768.0 * 180
        yaw = getSignInt16(data_buff[19] << 8 | data_buff[18]) / 32768.0 * 180
        send_hex_data(wt_imu, "FF AA 27 51 00")
    
    else:
        # print("///////////////////////////////////////////////////////////////////////////////")
        if buff[2] == 0x51:
            Q0 = getSignInt16(data_buff[5] << 8 | data_buff[4]) / 32768.0
            Q1 = getSignInt16(data_buff[7] << 8 | data_buff[6]) / 32768.0
            Q2 = getSignInt16(data_buff[9] << 8 | data_buff[8]) / 32768.0
            Q3 = getSignInt16(data_buff[11] << 8 | data_buff[10]) / 32768.0
            quaternion = [Q0, Q1, Q2, Q3]

            imu_msg.orientation.x = Q0
            imu_msg.orientation.y = Q1
            imu_msg.orientation.z = Q2
            imu_msg.orientation.w = Q3

        # Convert Euler Angles to Quaternion
        q = quaternion_from_euler(math.radians(roll), math.radians(pitch), math.radians(yaw))
        # imu_msg.orientation.x = q[0]
        # imu_msg.orientation.y = q[1]
        # imu_msg.orientation.z = q[2]
        # imu_msg.orientation.w = q[3]

        # Publish IMU message
        imu_pub.publish(imu_msg)

        # Create and publish roll angle
        roll_msg = Float32()
        roll_msg.data = roll
        euler_roll_pub.publish(roll_msg)

        # Create and publish pitch angle
        pitch_msg = Float32()
        pitch_msg.data = pitch
        euler_pitch_pub.publish(pitch_msg)

        # Create and publish yaw angle
        yaw_msg = Float32()
        yaw_msg.data = yaw
        euler_yaw_pub.publish(yaw_msg)

        # Create and publish Quaternion message
        quaternion_msg = Imu()
        quaternion_msg.header = Header()
        quaternion_msg.header.stamp = rospy.Time.now()
        quaternion_msg.header.frame_id = "base_imu"
        quaternion_msg.orientation.x = q[0]
        quaternion_msg.orientation.y = q[1]
        quaternion_msg.orientation.z = q[2]
        quaternion_msg.orientation.w = q[3]
        quaternion_pub.publish(quaternion_msg)

    # elif buff[1] == 0x3A:  # Magnetometer Data
    #     mag_msg = MagneticField()
    #     mag_msg.header = Header()
    #     mag_msg.header.stamp = rospy.Time.now()
    #     mag_msg.header.frame_id = "base_imu"

    #     # Magnetic field data
    #     mag_msg.magnetic_field.x = getSignInt16(data_buff[5] << 8 | data_buff[4]) / 120
    #     mag_msg.magnetic_field.y = getSignInt16(data_buff[7] << 8 | data_buff[6]) / 120
    #     mag_msg.magnetic_field.z = getSignInt16(data_buff[9] << 8 | data_buff[8]) / 120

    #     mag_pub.publish(mag_msg)

    buff.clear()  # Clear buff after processing
    key = 0  # Reset the key to 0

if __name__ == "__main__":
    rospy.init_node("imu_serial_node", anonymous=True)
    
    imu_pub = rospy.Publisher("/imu/data_raw", Imu, queue_size=10)
    mag_pub = rospy.Publisher("/imu/mag", MagneticField, queue_size=10)

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
    else:
        while not rospy.is_shutdown():
            try:
                buff_count = wt_imu.inWaiting()
            except Exception as e:
                rospy.logerr("IMU lost connection: " + str(e))
                exit(0)
            else:
                if buff_count > 0:
                    buff_data = wt_imu.read(buff_count)
                    for i in range(0, buff_count):
                        handleSerialData(buff_data[i])
