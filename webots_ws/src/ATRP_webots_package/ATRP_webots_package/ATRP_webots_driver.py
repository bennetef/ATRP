

"""ROS2 ATRP driver."""

import rclpy
from ackermann_msgs.msg import AckermannDrive
from rclpy.node import Node

PI = 3.14159265359

class ATRPDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        # ROS interface
        rclpy.init(args=None)
        self.__node = rclpy.create_node('atrp_node')

        self.__left_steer = self.__robot.getDevice('left_steer')
        self.__right_steer = self.__robot.getDevice('right_steer')
        self.__left_steer.setPosition(float('inf'))
        self.__left_steer.setVelocity(0)
        self.__right_steer.setPosition(float('inf'))
        self.__right_steer.setVelocity(0)

        self.__gps = self.__robot.getDevice('gps')
        self.__gps.enable(1000) # 1s/1Hz

        self.__logger = self.__node.get_logger()
        self.__node.create_subscription(AckermannDrive, 'cmd_ackermann', self.__cmd_ackermann_callback, 1)

    def __cmd_ackermann_callback(self, message):
        #self.__logger.info(f'rate: {self.__rate}')
        self.__left_steer.setVelocity(PI/10)
        self.__right_steer.setVelocity(PI/10)
        self.__robot.setCruisingSpeed(message.speed)
        self.__robot.setSteeringAngle(message.steering_angle)

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)