#include "controller.hpp"

namespace trajectory_tracking
{

Controller::Controller(): Node("controller") 
{
    RCLCPP_INFO(this->get_logger(), "Constructed");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
    tarcking_errors_publisher = this->create_publisher<std_msgs::msg::Float32MultiArray>("tracking_errors",10);
    robot_pose_publisher = this->create_publisher<geometry_msgs::msg::Vector3>("robot_pose",10);
    desired_pose_publisher = this->create_publisher<geometry_msgs::msg::Vector3>("desired_pose",10);


    _trajectory_handler = TrajectoryHandler();
    
    current_pose = MatrixXd(3,1);
    referance_pose = MatrixXd(3,1);
    tracking_error = MatrixXd(3,1);
    _rotation_matrix=MatrixXd(3,3);

    ref_linear_vel =0;
    ref_angular_vel=0;

    linear_vel_max=0.4;
    angular_vel_max = 0.8;

    _sampling_time = 0.01;
    _epsilon = 1.0;
    _b = 1.5; 
    current_time = 0.0;
    timer_ = this->create_wall_timer(0.01s, std::bind(&Controller::timerTry, this));

}



bool Controller::getCurrentPose(MatrixXd& _current_pose)
{
  geometry_msgs::msg::TransformStamped t;

  try {
    t = tf_buffer_->lookupTransform(
    "odom", "base_footprint",
    tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(
      this->get_logger(), "Could not transform %s to %s: %s",
      "base_footprint", "odom", ex.what());
    return false;

  }
  _current_pose(0,0) = t.transform.translation.x;
  _current_pose(1,0) = t.transform.translation.y;
  _current_pose(2,0) = quat2Yaw(t.transform.rotation);


  return true;

}


void Controller::timerTry()
{ 
   RCLCPP_INFO(this->get_logger(), "Current Time is %f",current_time);
  
  getCurrentPose(current_pose);
  _trajectory_handler.getReferanceTrajectoryAtTime(current_time ,referance_pose,ref_linear_vel,ref_angular_vel);

  // _rotation_matrix =        {{ cos(current_pose(2,0)), sin(current_pose(2,0)), 0},
  //                            {-sin(current_pose(2,0)), cos(current_pose(2,0)), 0},
  //                            {           0           ,           0           , 1}};

    
 
  _rotation_matrix(0,0) = cos(current_pose(2,0));
  _rotation_matrix(0,1) = sin(current_pose(2,0));
  _rotation_matrix(0,2) = 0;

  _rotation_matrix(1,0) = -sin(current_pose(2,0));
  _rotation_matrix(1,1) = cos(current_pose(2,0));
  _rotation_matrix(1,2) = 0;
  
  _rotation_matrix(2,0) = 0;
  _rotation_matrix(2,1) = 0;
  _rotation_matrix(2,2) = 1;

  std::cout << _rotation_matrix<< std::endl ;

  tracking_error =  _rotation_matrix* (referance_pose - current_pose);
  RCLCPP_INFO(this->get_logger(), "Referance  Theta Is : %f",referance_pose(2,0));
  RCLCPP_INFO(this->get_logger(), "Current  Theta Is : %f",current_pose(2,0));
  RCLCPP_INFO(this->get_logger(), "Tracking Erorr Theta Is : %f",tracking_error(2,0));
  // RCLCPP_INFO(this->get_logger(), "Refereance Pose X : %f Y: %f THT: %f", referance_pose(0,0),referance_pose(1,0),referance_pose(2,0));
  // RCLCPP_INFO(this->get_logger(), "Current Pose X : %f Y: %f THT: %f", current_pose(0,0),current_pose(1,0),current_pose(2,0));
  // RCLCPP_INFO(this->get_logger(), "Tracking Error  X : %f Y: %f THT: %f", tracking_error(0,0),tracking_error(1,0),tracking_error(2,0));

  double _a = sqrt(pow(ref_angular_vel,2) + (_b* pow(ref_linear_vel,2)));
  double set_linear_vel  = ref_linear_vel*cos(tracking_error(2,0)) + (2*_epsilon*_a) *tracking_error(0,0);
  double Ky = (_b*abs(ref_linear_vel));
  double Ktht = 2*_epsilon*_a;
  double set_angular_vel = ref_angular_vel + ((Ky*tracking_error(1,0)*sin(tracking_error(2,0)))/tracking_error(2,0)) + Ktht * tracking_error(2,0);


  // RCLCPP_INFO(this->get_logger(), "SET_ANGULAR_VEL IS %f",set_angular_vel);
  // RCLCPP_INFO(this->get_logger(), "SET_lin_VEL IS %f",set_linear_vel);

  saturateCommandVels(set_linear_vel,set_angular_vel);
  // RCLCPP_INFO(this->get_logger(), "SET_ANGULAR_VEL IS %f",set_angular_vel);
  // RCLCPP_INFO(this->get_logger(), "SET_lin_VEL IS %f",set_linear_vel);

  publishControlInputs(set_linear_vel,set_angular_vel);
  current_time = current_time + _sampling_time;
  publishErrors(tracking_error);
  publishPoses(current_pose,referance_pose);
}

double Controller::quat2Yaw(geometry_msgs::msg::Quaternion quat)
{
        tf2::Quaternion q;
        q.setX(quat.x);
        q.setY(quat.y);
        q.setZ(quat.z);
        q.setW(quat.w);

        tf2::Matrix3x3 mat_(q);
        double roll, pitch, yaw;
        mat_.getRPY(roll, pitch, yaw);

        if (yaw<0)
        {
          yaw = yaw + 2*3.14;
        }
        
        return yaw;

}


void Controller::saturateCommandVels(double& linear_vel ,double& angular_vel )
{ 
    std::vector<double> _sigma_list = {abs(linear_vel)/linear_vel_max , abs(angular_vel)/angular_vel_max,1};

    auto _max_index_iterator = std::max_element(std::begin(_sigma_list),std::end(_sigma_list));
    double _sigma = *_max_index_iterator;
    
    std::cout << "Max element is " << *_max_index_iterator
        << " at position " << std::distance(std::begin(_sigma_list), _max_index_iterator) << std::endl;
    

    switch(std::distance(std::begin(_sigma_list), _max_index_iterator)){

      case 0 :
            linear_vel =  (linear_vel/abs(linear_vel))*linear_vel_max;
            angular_vel = angular_vel/_sigma;
        break;

      case 1 :
            linear_vel = linear_vel / _sigma;
            angular_vel = (angular_vel/abs(angular_vel))*angular_vel_max;
        break;

      case 2 :
        break;


    }

  }


void Controller::publishControlInputs(double linear_vel , double angular_vel)
  {
      auto _msg = geometry_msgs::msg::Twist();
      _msg.linear.x = linear_vel;
      _msg.angular.z = angular_vel;

      cmd_vel_publisher->publish(_msg);

}

void Controller::publishErrors(MatrixXd _tracking_error){

    auto _x_error = std_msgs::msg::MultiArrayDimension();
    _x_error.label = "x_error";
    _x_error.size = 1;
    _x_error.stride =1;

    auto _y_error = std_msgs::msg::MultiArrayDimension();
    _y_error.label = "y_error";
    _y_error.size = 1;
    _y_error.stride =1;

    auto _theta_error = std_msgs::msg::MultiArrayDimension();
    _theta_error.label = "theta_error";
    _theta_error.size = 1;
    _theta_error.stride =1;

    auto _msg = std_msgs::msg::Float32MultiArray();
    _msg.layout.dim.push_back(_x_error);
    _msg.layout.dim.push_back(_y_error);
    _msg.layout.dim.push_back(_theta_error);

    _msg.data = {_tracking_error(0,0),_tracking_error(1,0),_tracking_error(2,0)};

    tarcking_errors_publisher->publish(_msg);



}

void Controller::publishPoses(MatrixXd robot_pose , MatrixXd desired_pose)
{
  auto robot_pose_msg = geometry_msgs::msg::Vector3();
  robot_pose_msg.x = robot_pose(0,0);
  robot_pose_msg.y = robot_pose(1,0);
  robot_pose_msg.z = robot_pose(2,0);
  

  auto desired_pose_msg = geometry_msgs::msg::Vector3();
  desired_pose_msg.x = desired_pose(0,0);
  desired_pose_msg.y = desired_pose(1,0);
  desired_pose_msg.z = desired_pose(2,0);

  robot_pose_publisher->publish(robot_pose_msg);
  desired_pose_publisher->publish(desired_pose_msg);

}


}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> nh = std::make_shared<trajectory_tracking::Controller>();
  rclcpp::spin(nh);
  rclcpp::shutdown();
  return 0;
}