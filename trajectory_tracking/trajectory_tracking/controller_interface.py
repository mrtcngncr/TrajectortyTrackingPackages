#!/usr/bin/env python3





from rclpy.action import ActionClient
from trajectory_tracking_msgs.action import TrajectoryTrack
from rclpy.callback_groups import ReentrantCallbackGroup

import time
import math

class TrajectoryTrackingControllerInterface():

    def __init__(self,node):
        

        self.node = node
        self.controller_action_client = ActionClient(self.node,TrajectoryTrack,"trajectory_tracking_action",callback_group=ReentrantCallbackGroup())
        self.isExecutionDone = False
        


    def execute_trajectory_async(self, trajectory):
        """ Send and execute trajectory asynchronous
            Arguments:
                trajectory : an TrajectoryTrackGoal msg               
            Returns:
                bool : Execute Request Succes
        """
        self.isExecutionDone = False

        isServerReady =self.controller_action_client.wait_for_server(2)

        if not isServerReady:
            self.node.get_logger().info('Controller Server Is Not Ready')
            return False

        self.__send_action_goal(trajectory)
        
        return True


    def send_circular_trajectory(self,duration,sampling_time=0.01,xc=1.5,yc=1.0,R=1.0,k=0.1):

        """ Send Circular Shape Trajectory To Controller
            Arguments:
                duration : moving time
                sampling_time : sampling_time for controller and trajectory
                xc : x value of center of circle
                yc : y value of center of circle
                R  : radius of circle
                k  : freq/speed constant of trajectory
            Retruns:
                None
        """
        goal = TrajectoryTrack.Goal()
        
        iterator = int(duration / sampling_time)
        t = 0
        for i in range(iterator):
           
            
            goal.trajectory.x.append(xc + R*math.sin(k*t))
            goal.trajectory.y.append(yc - R*math.cos(k*t))
            
            if(((k*t)%2*math.pi) > math.pi):
                goal.trajectory.theta.append(-2*math.pi + (k*t)%(2*math.pi))
            else:
                goal.trajectory.theta.append((k*t)%(2*math.pi))
            

            goal.trajectory.linear_vel.append(k*R)
            goal.trajectory.angular_vel.append(k)

            t += sampling_time

            
        goal.sampling_time = sampling_time
        self.execute_trajectory_async(goal)


    def eight_shape_trajectory(self,duration,sampling_time=0.01,xc=0,yc=0,R=1,k1=math.pi/50,k2=math.pi/25,k3=0):

        """ Send Eight Shape Trajectory To Controller
            Arguments:
                duration : moving time
                sampling_time : sampling_time for controller and trajectory
                xc : x value of center of eight-shape
                yc : y value of center of eight-shape
                R  : Coeffienct that effect eight shade width
                k1 : Coeffienct that effect eight shape slope through x
                k2 : Coeffienct that effect eight shape slope through y
                k3  : 0 or 1 must be set (Determine the starting direction
            Retruns:
                None
        """
    
        if not (k3==0 or k3 == 1):
            k3 = 0

        goal = TrajectoryTrack.Goal()       
        iterator = int(duration / sampling_time)
        t = 0

        for i in range(iterator):
            
            xDot = k1*R*math.cos(k1*t)
            yDot = k2*R*math.cos(k2*t)

            xDoubleDot = -k1*xDot
            yDoubleDot = -k2*yDot

            
            goal.trajectory.x.append(xc + R*math.sin(k1*t))        
            goal.trajectory.y.append(yc + R*math.sin(k2*t))
            goal.trajectory.theta.append(math.atan2(yDot,xDot)+ k3*math.pi)

            goal.trajectory.linear_vel.append(-math.sqrt(yDot**2 + xDot**2))
            goal.trajectory.angular_vel.append(((yDoubleDot*xDot)-(xDoubleDot*yDot)) / (yDot**2 + xDot**2))
            
            t += sampling_time

        goal.sampling_time = sampling_time
        self.execute_trajectory_async(goal)



    def __send_action_goal(self,trajectory):
        self._send_goal_future = self.controller_action_client.send_goal_async(trajectory)
        self._send_goal_future.add_done_callback(self.__goal_response_callback)

    def __goal_response_callback(self,future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info('Trajectory Execute Request Rejected')
            return

        self.node.get_logger().info('Trajectory Execute Request Accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.__get_result_callback)
    
    def __get_result_callback(self,future):
            result = future.result().result
            self.node.get_logger().info("in resul_callback")
            self.isExecutionDone = True





