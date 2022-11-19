#ifndef TRAJECTORY_TRACKING_CONTROLLER_HPP_
#define TRAJECTORY_TRACKING_CONTROLLER_HPP_

#include <chrono>
#include <memory>
#include <eigen3/Eigen/Core>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2/LinearMath/Quaternion.h>


#include "trajectory_tracking_msgs/action/trajectory_track.hpp"
#include "trajectory_tracking_msgs/srv/set_controller_parameters.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"


#include "trajectory_handler.hpp"




using Eigen::MatrixXd;
using namespace std::chrono_literals;
using  TrajectoryTrack = trajectory_tracking_msgs::action::TrajectoryTrack;

# define PI    3.141592653589793238462643383279502884L

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
      void calculateRotationMatrix(MatrixXd& rotation_matrix,MatrixXd current_pose_vec);
      void calculateControlInput(double& set_lin_vel , double& set_ang_vel);
      void setControllerParameters(double epsilon,double b);

      MatrixXd current_pose;
      MatrixXd referance_pose;
      MatrixXd tracking_error;
      MatrixXd _rotation_matrix ;
      double ref_linear_vel , ref_angular_vel;
      double linear_vel_max , angular_vel_max;
      double set_angular_vel , set_linear_vel;

      std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
      std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

      
      rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;

      //Debug and Visual Publishers
      rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr tarcking_errors_publisher;
      rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr robot_pose_publisher;
      rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr desired_pose_publisher;
      void publishErrors(MatrixXd _tracking_error);
      void publishPoses(MatrixXd robot_pose , MatrixXd desired_pose);

      //Action 
      rclcpp_action::Server<TrajectoryTrack>::SharedPtr  _controller_action_server;
      rclcpp_action::GoalResponse _controller_action_goal_handler(const rclcpp_action::GoalUUID & uuid,std::shared_ptr<const TrajectoryTrack::Goal> goal);
      rclcpp_action::CancelResponse controller_action_handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<TrajectoryTrack>> goal_handle);
      void _controller_aciton_handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<TrajectoryTrack>> goal_handle);
      void _controller_action_execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<TrajectoryTrack>> goal_handle);
      TrajectoryHandler _trajectory_handler;

      //Service
      rclcpp::Service<trajectory_tracking_msgs::srv::SetControllerParameters>::SharedPtr set_controller_params_service;
      void set_controller_params_service_cv(const std::shared_ptr<trajectory_tracking_msgs::srv::SetControllerParameters::Request> request,
                                                  std::shared_ptr<trajectory_tracking_msgs::srv::SetControllerParameters::Response>);
 
    
      rclcpp::TimerBase::SharedPtr timer_{nullptr};
      //Controller Parameters
      double _sampling_time, _epsilon , _b , current_time;






};












}


#endif