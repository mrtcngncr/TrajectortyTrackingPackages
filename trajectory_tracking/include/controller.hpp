#ifndef TRAJECTORY_TRACKING_CONTROLLER_HPP_
#define TRAJECTORY_TRACKING_CONTROLLER_HPP_

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2/LinearMath/Quaternion.h>



#include <eigen3/Eigen/Core>

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "std_msgs/msg/float32_multi_array.hpp"



#include "trajectory_handler.hpp"

using Eigen::MatrixXd;
using namespace std::chrono_literals;


namespace trajectory_tracking
{


class Controller : public rclcpp::Node 
{
    
    
    public:
      Controller();

      bool getCurrentPose(MatrixXd& _current_pose);
      bool getReferancePose(MatrixXd& _referance_pose);


    private:
      void timerTry();
      double quat2Yaw(geometry_msgs::msg::Quaternion quat);
      void saturateCommandVels(double& linear_vel ,double& angular_vel );
      void publishControlInputs(double linear_vel , double angular_vel);
      void publishErrors(MatrixXd _tracking_error);
      void publishPoses(MatrixXd robot_pose , MatrixXd desired_pose);

      MatrixXd current_pose;
      MatrixXd referance_pose;
      MatrixXd tracking_error;
      MatrixXd _rotation_matrix ;
      double ref_linear_vel , ref_angular_vel;
      double linear_vel_max , angular_vel_max;

      std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
      std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

      rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
      rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr tarcking_errors_publisher;
      rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr robot_pose_publisher;
      rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr desired_pose_publisher;
      
      

      
      TrajectoryHandler _trajectory_handler;

      rclcpp::TimerBase::SharedPtr timer_{nullptr};
      //Controller Parameters
      double _sampling_time, _epsilon , _b , current_time;






};












}


#endif