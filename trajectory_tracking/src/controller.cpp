#include "controller.hpp"

namespace trajectory_tracking
{

Controller::Controller(): Node("controller") 
{
    using namespace std::placeholders;

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
    tarcking_errors_publisher = this->create_publisher<std_msgs::msg::Float32MultiArray>("tracking_errors",10);
    robot_pose_publisher = this->create_publisher<geometry_msgs::msg::Vector3>("robot_pose",10);
    desired_pose_publisher = this->create_publisher<geometry_msgs::msg::Vector3>("desired_pose",10);

    _controller_action_server = rclcpp_action::create_server<TrajectoryTrack>(
      this,
      "trajectory_tracking_action",
      std::bind(&Controller::_controller_action_goal_handler,this,_1,_2),
      std::bind(&Controller::controller_action_handle_cancel,this,_1),
      std::bind(&Controller::_controller_aciton_handle_accepted,this,_1));

    set_controller_params_service = this->create_service<trajectory_tracking_msgs::srv::SetControllerParameters>(
                                          "set_controller_parameters",
                                          [this](const std::shared_ptr<trajectory_tracking_msgs::srv::SetControllerParameters::Request> request,
                                                  std::shared_ptr<trajectory_tracking_msgs::srv::SetControllerParameters::Response> response){return set_controller_params_service_cv(request,response);});

   
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
    _epsilon = 10.0;
    _b = 10.0; 
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
   
  std::cout << "epsilon is " << _epsilon << std::endl;
  std::cout << "B is " << _b << std::endl;
  
  // getCurrentPose(current_pose);
  // _trajectory_handler.getCircularReferanceTrajectoryAtTime(current_time ,referance_pose,ref_linear_vel,ref_angular_vel);
  // _trajectory_handler.getEightShapeReferanceTrajectoryAtTime(current_time ,referance_pose,ref_linear_vel,ref_angular_vel,0.01);

     
  // _rotation_matrix(0,0) = cos(current_pose(2,0));
  // _rotation_matrix(0,1) = sin(current_pose(2,0));
  // _rotation_matrix(0,2) = 0;

  // _rotation_matrix(1,0) = -sin(current_pose(2,0));
  // _rotation_matrix(1,1) = cos(current_pose(2,0));
  // _rotation_matrix(1,2) = 0;
  
  // _rotation_matrix(2,0) = 0;
  // _rotation_matrix(2,1) = 0;
  // _rotation_matrix(2,2) = 1;

  // tracking_error =  _rotation_matrix* (referance_pose - current_pose);


  // if(abs(tracking_error(2,0))>(2*PI-0.5))
  // {
  //   tracking_error(2,0) = -1*(tracking_error(2,0)/abs(tracking_error(2,0)))* (2*PI-abs(tracking_error(2,0)));
  // }
  // // RCLCPP_INFO(this->get_logger(), "Referance  Theta Is : %f",referance_pose(2,0));
  // // RCLCPP_INFO(this->get_logger(), "Current  Theta Is : %f",current_pose(2,0));
  // // RCLCPP_INFO(this->get_logger(), "Tracking Erorr Theta Is : %f",tracking_error(2,0));
  // // RCLCPP_INFO(this->get_logger(), "Refereance Pose X : %f Y: %f THT: %f", referance_pose(0,0),referance_pose(1,0),referance_pose(2,0));
  // // RCLCPP_INFO(this->get_logger(), "Current Pose X : %f Y: %f THT: %f", current_pose(0,0),current_pose(1,0),current_pose(2,0));
  // // RCLCPP_INFO(this->get_logger(), "Tracking Error  X : %f Y: %f THT: %f", tracking_error(0,0),tracking_error(1,0),tracking_error(2,0));

  // double _a = sqrt(pow(ref_angular_vel,2) + (_b* pow(ref_linear_vel,2)));
  // double set_linear_vel  = ref_linear_vel*cos(tracking_error(2,0)) + (2*_epsilon*_a) *tracking_error(0,0);
  // double Ky = (_b*abs(ref_linear_vel));
  // double Ktht = 2*_epsilon*_a;
  // double set_angular_vel = ref_angular_vel + ((Ky*tracking_error(1,0)*sin(tracking_error(2,0)))/tracking_error(2,0)) + Ktht * tracking_error(2,0);
  // // RCLCPP_INFO(this->get_logger(), "REF ANGULAR VEL IS %f",ref_angular_vel);
  // // RCLCPP_INFO(this->get_logger(), "set ANGULAR VEL IS %f",set_angular_vel);


  // saturateCommandVels(set_linear_vel,set_angular_vel);
  // // RCLCPP_INFO(this->get_logger(), "SET_ANGULAR_VEL IS %f",set_angular_vel);
  // // RCLCPP_INFO(this->get_logger(), "SET_lin_VEL IS %f",set_linear_vel);

  // publishControlInputs(set_linear_vel,set_angular_vel);
  current_time = current_time + _sampling_time;
  // publishErrors(tracking_error);
  // publishPoses(current_pose,referance_pose);
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

        // if (yaw<0)
        // {
        //   yaw = yaw + 2*PI;
        // }
        
        return yaw;

}

void Controller::saturateCommandVels(double& linear_vel ,double& angular_vel )
{ 
    std::vector<double> _sigma_list = {abs(linear_vel)/linear_vel_max , abs(angular_vel)/angular_vel_max,1};

    auto _max_index_iterator = std::max_element(std::begin(_sigma_list),std::end(_sigma_list));
    double _sigma = *_max_index_iterator;
    
    // std::cout << "Max element is " << *_max_index_iterator
    //     << " at position " << std::distance(std::begin(_sigma_list), _max_index_iterator) << std::endl;
    

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

void Controller::calculateRotationMatrix(MatrixXd& rotation_mtrx, MatrixXd current_pose_vec){
  rotation_mtrx(0,0) = cos(current_pose_vec(2,0));
  rotation_mtrx(0,1) = sin(current_pose_vec(2,0));
  rotation_mtrx(0,2) = 0;

  rotation_mtrx(1,0) = -sin(current_pose_vec(2,0));
  rotation_mtrx(1,1) = cos(current_pose_vec(2,0));
  rotation_mtrx(1,2) = 0;
  
  rotation_mtrx(2,0) = 0;
  rotation_mtrx(2,1) = 0;
  rotation_mtrx(2,2) = 1;


}

void Controller::calculateControlInput(double& set_lin_vel , double& set_ang_vel)
{
  double _a = sqrt(pow(ref_angular_vel,2) + (_b* pow(ref_linear_vel,2)));
  double Ky = (_b*abs(ref_linear_vel));
  double Ktht = 2*_epsilon*_a;
  set_ang_vel = ref_angular_vel + ((Ky*tracking_error(1,0)*sin(tracking_error(2,0)))/tracking_error(2,0)) + Ktht * tracking_error(2,0);
  set_lin_vel  = ref_linear_vel*cos(tracking_error(2,0)) + (2*_epsilon*_a) *tracking_error(0,0);

}

void Controller::setControllerParameters(double epsilon,double b)
{
  _epsilon = epsilon;
  _b = b;

}


rclcpp_action::GoalResponse Controller::_controller_action_goal_handler(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const TrajectoryTrack::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received Trajectory");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void Controller::_controller_aciton_handle_accepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<TrajectoryTrack>> goal_handle)
{
  using namespace std::placeholders;
  std::thread{std::bind(&Controller::_controller_action_execute, this, _1), goal_handle}.detach();
}

rclcpp_action::CancelResponse Controller::controller_action_handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<TrajectoryTrack>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Trajecrory Tracking Canceled");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}


void Controller::_controller_action_execute(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<TrajectoryTrack>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Trajecrory Tracking Executed");
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<TrajectoryTrack::Result>();

  RCLCPP_INFO(this->get_logger(), "iter %ld",goal->trajectory.x.size());

  for (size_t i = 0; i < goal->trajectory.x.size()  && rclcpp::ok() ; i++)
  {
    auto start = std::chrono::system_clock::now();

    if (goal_handle->is_canceling()) {
        result->succes = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Trajectort Execution Canceled");
        return;
      }

    getCurrentPose(current_pose);
    
    referance_pose(0,0) = goal->trajectory.x[i] ;
    referance_pose(1,0) = goal->trajectory.y[i] ;
    referance_pose(2,0) = goal->trajectory.theta[i]; 

    ref_linear_vel = goal->trajectory.linear_vel[i];
    ref_angular_vel = goal->trajectory.angular_vel[i];
    calculateRotationMatrix(_rotation_matrix,current_pose);

    tracking_error =  _rotation_matrix* (referance_pose - current_pose);

    std::cout << "for  current state in action"<< std::endl ;
    std::cout << current_pose<< std::endl ;
    std::cout << "Referance Pose"<< std::endl ;
    std::cout << referance_pose<< std::endl ;

    //This part for avoiding error jump due to atan2 angle mapping 
    if(abs(tracking_error(2,0))>(PI))
    {
      tracking_error(2,0) = -1*(tracking_error(2,0)/abs(tracking_error(2,0)))* (2*PI-abs(tracking_error(2,0)));
    }

    calculateControlInput(set_linear_vel,set_angular_vel);
    saturateCommandVels(set_linear_vel,set_angular_vel);

    publishControlInputs(set_linear_vel,set_angular_vel);

    publishErrors(tracking_error);
    publishPoses(current_pose,referance_pose);





    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = end - start;
    if (diff.count()<goal->sampling_time)
    {
      std::this_thread::sleep_for(std::chrono::duration<double>(goal->sampling_time - diff.count()));
    }
    end = std::chrono::system_clock::now();
    diff = end - start;
    std::cout<<diff.count()<<std::endl;
  }
  


  if (rclcpp::ok()) 
  {
      result->succes = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Trajectory Completed");
  }
}

void Controller::set_controller_params_service_cv(const std::shared_ptr<trajectory_tracking_msgs::srv::SetControllerParameters::Request> request,
                                                  std::shared_ptr<trajectory_tracking_msgs::srv::SetControllerParameters::Response>)
{
    setControllerParameters(request->epsilon,request->b);
    return;
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