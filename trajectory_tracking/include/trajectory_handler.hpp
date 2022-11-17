#ifndef TRAJECTORY_TRACKING_TRAJECTORY_HANDLER_HPP_
#define TRAJECTORY_TRACKING_TRAJECTORY_HANDLER_HPP_

#include <eigen3/Eigen/Core>

using Eigen::MatrixXd;
# define PI    3.141592653589793238462643383279502884L
namespace trajectory_tracking
{

class TrajectoryHandler
{

    public:
        TrajectoryHandler();

        void getCircularReferanceTrajectoryAtTime(double time , MatrixXd& _referance_pose, double& _reference_linear_vel , double& _referance_angular_vel);
        void getEightShapeReferanceTrajectoryAtTime(double time , MatrixXd& _referance_pose, double& _reference_linear_vel , double& _referance_angular_vel,double sampling_time);


    
    private:
    
        double _R , _k , _xc , _yc;

        double _k1 ,_k2 ,_k3 , _xc2, _yc2;




};

TrajectoryHandler::TrajectoryHandler()
{   
    _R =1;
    _k =0.1;
    _xc =1.5;
    _yc = 1;


    _k1 = (2*PI)/100;
    _k2 = 2*_k1;
    _k3 =0;

    _xc2 = 0;
    _yc2 = 0;

    

    

}

void TrajectoryHandler::getCircularReferanceTrajectoryAtTime(double time , MatrixXd& _referance_pose, double& _reference_linear_vel , double& _referance_angular_vel)
{
    _referance_pose(0,0) = _xc + _R*sin(_k*time);
    _referance_pose(1,0) = _yc - _R*cos(_k*time);
    _referance_pose(2,0) = fmod(_k*time,2*PI);
    
    if (_referance_pose(2,0)>PI);
    {
        _referance_pose(2,0) = -2*PI +_referance_pose(2,0) ;
    }

    _reference_linear_vel = _k*_R;
    _referance_angular_vel = _k;

}


void TrajectoryHandler::getEightShapeReferanceTrajectoryAtTime(double time , MatrixXd& _referance_pose, double& _reference_linear_vel , double& _referance_angular_vel , double sampling_time)
{


    _referance_pose(0,0) =  _xc2 + _R*sin(_k1*time);    
    _referance_pose(1,0) =  _yc2 + _R*sin(_k2*time);  
    
    double xDot = _k1*_R*cos(_k1*time);
    double yDot = _k2*_R*cos(_k2*time);

    double xDoubleDot = -_k1*xDot;
    double yDoubleDot = -_k2*yDot;


    double _refTheta =  atan2(yDot,xDot) + _k3*PI;

    _referance_pose(2,0) = _refTheta;
 
    _reference_linear_vel = -sqrt(pow(yDot,2)+pow(xDot,2));   
    _referance_angular_vel = ((yDoubleDot*xDot)-(xDoubleDot*yDot))/ (pow(yDot,2)+pow(xDot,2));

}


}









#endif