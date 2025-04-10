#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
from math import sin, cos
import time



import smbus 			#import SMBus module of I2C
from time import sleep          #import
import mpu6050

#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

device_address = 0x68   # MPU6050 device address


class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 10Hz
        self.get_logger().info('IMU Publisher Node has been started')
        
       

        #self.bus = smbus.SMBus(1)

        #self.MPU_Init()
        self.mpu6050 = mpu6050.mpu6050(0x68)


    def MPU_Init(self):
        #write to sample rate register
        self.bus.write_byte_data(device_address, SMPLRT_DIV, 7)
        
        #Write to power management register
        self.bus.write_byte_data(device_address, PWR_MGMT_1, 1)
        
        #Write to Configuration register
        self.bus.write_byte_data(device_address, CONFIG, 0)
        
        #Write to Gyro configuration register
        self.bus.write_byte_data(device_address, GYRO_CONFIG, 24)
        
        #Write to interrupt enable register
        self.bus.write_byte_data(device_address, INT_ENABLE, 1)

    def read_raw_data(self, addr):
	    #Accelero and Gyro value are 16-bit
        high = self.bus.read_byte_data(device_address, addr)
        low = self.bus.read_byte_data(device_address, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value

    def read_sensor_data(self):
        # Read the accelerometer values
        accelerometer_data = self.mpu6050.get_accel_data()

        # Read the gyroscope values
        gyroscope_data = self.mpu6050.get_gyro_data()

        # Read temp
        temperature = self.mpu6050.get_temp()

        return accelerometer_data, gyroscope_data, temperature

    def timer_callback(self):
        # Create IMU message
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_frame"
        
        accelerometer_data, gyroscope_data, temperature = self.read_sensor_data()
        # Angular velocity (using scaled values)
        msg.angular_velocity.x = gyroscope_data['x']
        msg.angular_velocity.y = gyroscope_data['y']
        msg.angular_velocity.z = gyroscope_data['z']
        msg.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        
        # # Linear acceleration (using scaled values)
        msg.linear_acceleration.x = accelerometer_data['x']
        msg.linear_acceleration.y = accelerometer_data['y']
        msg.linear_acceleration.z = accelerometer_data['z']
        msg.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        
        # # Publish message
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    imu_publisher = ImuPublisher()
    
    try:
        rclpy.spin(imu_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        imu_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()