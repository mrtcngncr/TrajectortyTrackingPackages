#ifndef TRAJECTORY_TRACKING_TRAJECTORY_HANDLER_HPP_
#define TRAJECTORY_TRACKING_TRAJECTORY_HANDLER_HPP_

#include <eigen3/Eigen/Core>

using Eigen::MatrixXd;

namespace trajectory_tracking
{

class TrajectoryHandler
{

    public:
        TrajectoryHandler();

        void getReferanceTrajectoryAtTime(double time , MatrixXd& _referance_pose, double& _reference_linear_vel , double& _referance_angular_vel);


    
    private:
    
        double _R , _k , _xc , _yc;




};

TrajectoryHandler::TrajectoryHandler()
{   
    _R =1;
    _k =0.1;
    _xc =0.0;
    _yc = 1;
    

}

void TrajectoryHandler::getReferanceTrajectoryAtTime(double time , MatrixXd& _referance_pose, double& _reference_linear_vel , double& _referance_angular_vel)
{
    _referance_pose(0,0) = _xc + _R*sin(_k*time);
    _referance_pose(1,0) = _yc - _R*cos(_k*time);
    _referance_pose(2,0) = fmod(_k*time,6.28);
    

    _reference_linear_vel = _k*_R;
    _referance_angular_vel = _k;

}





}









#endif