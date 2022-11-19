#!/usr/bin/env python3

import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from trajectory_tracking.controller_interface import TrajectoryTrackingControllerInterface

from trajectory_tracking_msgs.action import TrajectoryTrack



class Test():

    def __init__(self) :
        
        self.nh = Node("Test")
        self.interface = TrajectoryTrackingControllerInterface(self.nh)

        self.interface.eight_shape_trajectory(150)
            
        self.nh.get_logger().info("sended")
        thrd = threading.Thread(self.handle_ros_spin(),daemon=True)
        thrd.start()

       

    def handle_ros_spin(self):
        self.nh.get_logger().info("asdasdasd")
        executor = MultiThreadedExecutor()
        executor.add_node(self.nh)
        executor.spin()



rclpy.init()

test = Test()




