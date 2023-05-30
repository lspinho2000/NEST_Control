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
//#include "pid.h"
#include "nest_func.hpp"


#include <sstream>

#define pi 3.1415926535921

//bool armed;

ros::Publisher drone_vel;
ros::Publisher follow_point;

geometry_msgs::Pose point_foll;


geometry_msgs::Pose zarco_point(const nav_msgs::OdometryConstPtr &msg, double yaw, double x, double y){  //Function that defines a point to be followed that will be shown in world frame

  //Initializes the variables to be used to calculate the point
  double x_point, y_point;

  //Transforms the point from global frame to local boat frame
  x_point = msg->pose.pose.position.x - (x*cos(yaw) - y*sin(yaw));
  y_point = msg->pose.pose.position.y - (x*sin(yaw) + y*cos(yaw));

  //Creates the pose message to store the point obtained
  geometry_msgs::Pose res;
  res.position.x = x_point;
  res.position.y = y_point;
  res.position.z = msg->pose.pose.position.z;

  res.orientation = msg->pose.pose.orientation;

  //Returns the pose message
  return res;
}


void drone_pub(const geometry_msgs::Twist &msg){
  drone_vel.publish(msg);
}

void zarco_speed(const nav_msgs::OdometryConstPtr &msg)
{
   double yaw;
   yaw = tf::getYaw(msg->pose.pose.orientation);


//   double angle;
//   angle = yaw*180/pi;

   double x_speed,y_speed,z_speed;

   x_speed = msg->twist.twist.linear.x * cos(yaw) + msg->twist.twist.linear.y * sin(yaw);
   y_speed = msg->twist.twist.linear.x * (-sin(yaw)) + msg->twist.twist.linear.y*cos(yaw);
   z_speed = msg->twist.twist.linear.z;


   point_foll = zarco_point(msg,yaw,5,0);

   follow_point.publish(point_foll);


}


void drone_cb(const nav_msgs::OdometryConstPtr &msg){

  double tmp_x,tmp_y;
  tmp_x = msg->pose.pose.position.y;
  tmp_y = -(msg->pose.pose.position.x);

  double result_pid_x, result_pid_y;
  PID pid_x;
  pid_x.kp = kp_x;
  pid_x.ki = ki_x;
  pid_x.kd = kd_x;
  result_pid_x = pid_x.calculate(point_foll.position.x,tmp_x);

  PID pid_y;
  pid_y.kp = kp_y;
  pid_y.kd = kd_y;
  pid_y.ki = ki_y;
  result_pid_y = pid_y.calculate(point_foll.position.y, tmp_y);



  double yaw;
  yaw = tf::getYaw(msg->pose.pose.orientation) - pi/2;

  yaw = NormalizeAngle(yaw);


  double yaw_zarco;
  yaw_zarco = tf::getYaw(point_foll.orientation);


 double angle = atan2(sin(yaw_zarco), cos(yaw_zarco)) - atan2(sin(yaw), cos(yaw));

 angle = NormalizeAngle(angle);


  geometry_msgs::Twist drone_msg;

  if(abs(result_pid_x) < 0.01){
    result_pid_x = 0;
  }
  if(abs(result_pid_y) < 0.01){
    result_pid_y = 0;
  }

  drone_msg.linear.x = result_pid_x * cos(yaw) + result_pid_y * sin(yaw);
  drone_msg.linear.y = result_pid_x * (-sin(yaw)) + result_pid_y * cos(yaw);
  drone_msg.angular.z = angle;

  drone_pub(drone_msg);



  ROS_WARN_STREAM("Resultado do PID em X: " << result_pid_x << "Resultado do PID em Y: " << result_pid_y);


}

int main(int argc, char **argv){
  ros::init(argc,argv,"nest_control");
  ros::NodeHandle n;
  ros::NodeHandle nh;

  drone_vel = nh.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 1);
  ros::Subscriber sub_zarco = n.subscribe("/fw_asv0/p3d_odom", 1, zarco_speed);

  follow_point = nh.advertise<geometry_msgs::Pose>("/fw_asv0/follow_point",1);

  ros::Subscriber sub_drone = n.subscribe("/mavros/local_position/odom",1,drone_cb);


  ros::spin();

  return 0;
}
