#ifndef NEST_FUNC_HPP
#define NEST_FUNC_HPP

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cmath>
#include <geometry_msgs/Pose.h>


#define pi 3.1415926535921

//geometry_msgs::Pose point_foll;

double kp_x = 0.5, ki_x = 0.00005, kd_x = 0.0;
double kp_y = 0.5, ki_y = 0.00005, kd_y = 0.0;
//double kp_angle = 1.0, ki_angle = 0.0, kd_angle = 0.0;

//PID class

class PID{
public:
  double kp, ki, kd;
  //kp constant influences the proportional aspect
  //ki constant influences the integral aspect
  //kd constant influences the derivative aspect


  double calculate(double target, double measure){

    error = target - measure;

    //Integral aspect is the sum of all errors obtained in the loop
    integral += error;

    //Anti-windup mechanism

    if (integral > 25){
        integral = 25;
    }
    else if (integral < -25){
        integral = -25;
    }


    //Derivative aspect is the difference between current error and last error
    derivative = error - previousError;

    //The output is the sum of all aspects multiplied by their corresponding constants
    output = kp * error + ki * integral - kd * derivative;

    previousError = error;

    //Returns the output
    return output;
  }

private:
  double error,integral,derivative,output,previousError;
  //Here the value of each calculated value is stored privately
};


//Normalizes an angle between the range of [-pi;pi]
double NormalizeAngle(double yaw){
  if(yaw>pi){
    yaw -= 2*pi;
  }else if (yaw <= -pi){
    yaw += 2*pi;
  }

  return yaw;
}


//  mavros_msgs::SetMode srv;
//  ros::ServiceClient client = nh.serviceClient<const mavros_msgs::SetMode>("/mavros/set_mode");


//  if(armed != true){
//    //srv.request.base_mode = mavros_msgs::SetMode::Request::MAV_MODE_GUIDED_ARMED;
//    srv.request.custom_mode = 'GUIDED';
//    //srv.request.MAV_MODE_GUIDED_ARMED;
//  }

//  if (client.call(srv))
//    {
//      ROS_INFO_STREAM("Armed : " << srv.response.mode_sent);

//    }
//    else
//    {
//      ROS_ERROR("Failed to call service arm");

//    }



#endif // NEST_FUNC_HPP
