#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3 ,Twist
from std_srvs.srv import Trigger


import csv
from datetime import datetime
import pandas as pd
import os
class DataSaver(Node):
    
    def __init__(self):
        super().__init__("data_saver")

        self.create_subscription(Float32MultiArray,"tracking_errors",self.save_error_data,10)
        self.create_subscription(Vector3,"robot_pose",self.save_robot_pose_data,10)
        self.create_subscription(Vector3,"desired_pose",self.save_desired_pose_data,10)
        self.create_subscription(Twist,"cmd_vel",self.save_cmd_vel_data,10)

        self.create_service(Trigger,"save_data",self.save_data)

        self.data = {
                "x_error":[],
                "y_error":[],
                "theta_error":[],

                "current_pose_x":[],
                "current_pose_y":[],

                "desired_pose_x":[],
                "desired_pose_y":[],

                "linear_vel":[],
                "angular_vel":[]

        }
    
    def save_error_data(self,msg):
        
        self.data["x_error"].append(msg.data[0])
        self.data["y_error"].append(msg.data[1])
        self.data["theta_error"].append(msg.data[2])
        print(type(msg.data[0]))
              
    def save_robot_pose_data(self,msg):
        self.data["current_pose_x"].append(msg.x)
        self.data["current_pose_y"].append(msg.y)

    def save_desired_pose_data(self,msg):
        self.data["desired_pose_x"].append(msg.x)
        self.data["desired_pose_y"].append(msg.y)

    def save_cmd_vel_data(self,msg):
        self.data["linear_vel"].append(msg.linear.x)
        self.data["angular_vel"].append(msg.angular.z)

    def save_data(self,req,resp):

        now = datetime.now()
        path = self.find_data_folder_path()

        
        df = pd.DataFrame(self.data)
        df.to_csv(path + "/testresults" + str(now) + ".csv" , index=False, header=True)

        return resp

    def find_data_folder_path(self):

        _p = os.path.realpath(__file__)
        abs_path_list = _p.split(os.sep)

        path= ""

        for _folders in abs_path_list:
            
            if _folders == "install":
                break

            path += _folders + "/"

        path += "src/TrajectortyTrackingPackages/trajectory_tracking_test/datas"
        
        return path
            
            

rclpy.init()
saver = DataSaver()

rclpy.spin(saver)

saver.destroy_node()
rclpy.shutdown()