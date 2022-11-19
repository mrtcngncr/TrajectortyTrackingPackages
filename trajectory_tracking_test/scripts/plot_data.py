#!/usr/bin/env python3

import matplotlib.pyplot as plt
import csv
import os
import pandas as pd

def find_data_folder_path():

    _p = os.path.realpath(__file__)
    abs_path_list = _p.split(os.sep)

    path= ""

    for _folders in abs_path_list:
        
        if _folders == "scripts":
            break

        path += _folders + "/"

    path += "datas"
    
    return path
        

def import_data(data_name):

    folder_path = find_data_folder_path()

    data_path =  folder_path + "/" + data_name

    df = pd.read_csv(data_path)

    data = {
        "x_error":df["x_error"].tolist(),
        "y_error":df["y_error"].tolist(),
        "theta_error":df["theta_error"].tolist(),
        "current_pose_x":df["current_pose_x"].tolist(),
        "current_pose_y":df["current_pose_y"].tolist(),
        "desired_pose_x":df["desired_pose_x"].tolist(),
        "desired_pose_y":df["desired_pose_y"].tolist(),
        "linear_vel":df["linear_vel"].tolist(),
        "angular_vel":df["angular_vel"].tolist()


    }

    
    return data


def print_errors(data):

    plt.figure()

    error_x = data["x_error"]
    error_y = data["y_error"]
    error_theta = data["theta_error"]
    t = create_time_array(error_x)
    
    plt.plot(t,error_theta,  label="theta_error")
    plt.plot(t,error_x ,label="x_error")
    plt.plot(t,error_y, label= "y_error")
    plt.legend()
  

def create_time_array(data):

    sampling_time = 0.01

    time_list = []
    _time = 0

    for i in range(len(data)):
        time_list.append(_time)
        _time += sampling_time

    return time_list
    

def print_poses(data):
    
    plt.figure()
    real_pose_x = data["current_pose_x"]
    real_pose_y = data["current_pose_y"]
    
    desired_pose_x = data["desired_pose_x"]
    desired_pose_y = data["desired_pose_y"]

    print(real_pose_x)
    plt.plot(real_pose_x,real_pose_y, label="robot_trajectory")
    plt.plot(desired_pose_x,desired_pose_y,label="desired_trajectory")
    plt.legend()

    

def print_vels(data):
    plt.figure()

    linear_vel = data["linear_vel"]
    angular_vel = data["angular_vel"]
    t = create_time_array(linear_vel)

    plt.plot(t,linear_vel,label="linear_vel")
    plt.plot(t,angular_vel,label="angular_vel")
    plt.legend()
    

data = import_data("testresults2022-11-19 22:56:42.262156.csv")



print_poses(data)
print_errors(data)
print_vels(data)

plt.show()
