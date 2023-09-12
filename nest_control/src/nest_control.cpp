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
#include <mavros_msgs/RCIn.h>
#include <cmath>
//#include "pid.h"
#include "nest_func.hpp"


#include <sstream>

#define pi 3.1415926535921

PID pid_x;
PID pid_y;

Button Button_press_loiter;
Button Button_press_vessel;


ros::Publisher drone_vel;
ros::Publisher follow_point;

geometry_msgs::Pose point_foll;

std::string state;
bool flag_loiter = false;
bool flag_vessel = false;

bool button_loiter = false;
bool button_vessel = false;


void cb_state(const mavros_msgs::State::ConstPtr &msg){
    state = msg->mode;
}


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


void vessel_following(const nav_msgs::OdometryConstPtr &msg){

  if(state == "GUIDED" && button_vessel){
        flag_vessel = true;
  }else{
        flag_vessel = false;
  }
    
    
  if(flag_vessel){
    double tmp_x,tmp_y;
    tmp_x = msg->pose.pose.position.y;
    tmp_y = -(msg->pose.pose.position.x);

    double result_pid_x, result_pid_y;
    result_pid_x = pid_x.calculate(point_foll.position.x,tmp_x);
    result_pid_y = pid_y.calculate(point_foll.position.y, tmp_y);



      double yaw;
    yaw = tf::getYaw(msg->pose.pose.orientation) - pi/2;

      yaw = NormalizeAngle(yaw);


      double yaw_zarco;
      yaw_zarco = tf::getYaw(point_foll.orientation);


     double angle = atan2(sin(yaw_zarco), cos(yaw_zarco)) - atan2(sin(yaw), cos(yaw));

     angle = NormalizeAngle(angle);


      geometry_msgs::Twist drone_msg;

      if(abs(result_pid_x) < 1){
        result_pid_x = 0;
      }
      if(abs(result_pid_y) < 1){
        result_pid_y = 0;
      }

      drone_msg.linear.x = result_pid_x * cos(yaw) + result_pid_y * sin(yaw);
      drone_msg.linear.y = result_pid_x * (-sin(yaw)) + result_pid_y * cos(yaw);
      drone_msg.angular.z = angle;

      drone_pub(drone_msg);



      ROS_WARN_STREAM("Resultado do PID em X: " << result_pid_x << "Resultado do PID em Y: " << result_pid_y);
    }

}


void NEST_loiter(const geometry_msgs::PoseStamped::ConstPtr &msg){
    geometry_msgs::PoseStamped loiter_point;
    geometry_msgs::Twist NEST_speed;
    
    double angle, yaw, yaw_curr;
    double result_pidX, result_pidY;
    
    
    if(state == "GUIDED" && !button_vessel && button_loiter){
        flag_loiter = true;
    }else{
        loiter_point.pose.position = msg->pose.position;
        loiter_point.pose.orientation = msg->pose.orientation; 
        flag_loiter = false;
    }
    
    if(flag_loiter){
        result_pidX = pid_x.calculate(loiter_point.pose.position.x,msg->pose.position.x);
        result_pidY = pid_y.calculate(loiter_point.pose.position.y,msg->pose.position.y);
        yaw_curr = tf::getYaw(msg->pose.orientation);
        yaw_curr = NormalizeAngle(yaw_curr);
        yaw = tf::getYaw(loiter_point.pose.orientation);
        yaw = NormalizeAngle(yaw);
        
        if(abs(result_pidX) <  1){
            result_pidX = 0;
        }
        if(abs(result_pidY) < 1){
            result_pidY = 0;
        }
        
        angle = atan2(sin(yaw),cos(yaw)) - atan2(sin(yaw_curr),cos(yaw_curr));
        angle = NormalizeAngle(angle);
        
        NEST_speed.linear.x = result_pidX;
        NEST_speed.linear.y = result_pidY;
        NEST_speed.angular.z = 0.5*angle;
        
        drone_pub(NEST_speed);
        
        ROS_WARN_STREAM("Linear Speed (X)  = " << result_pidX << "Linear speed (Y) = " << result_pidY);
        ROS_WARN_STREAM("Angular Speed (yaw) = " << angle);
        ROS_WARN_STREAM(" Difference : " << "X = Curr/Des : " << msg->pose.position.x << loiter_point.pose.position.x << "Y = Curr/Des : " << msg->pose.position.y << loiter_point.pose.position.y);
    }
}

void cb_button(const mavros_msgs::RCIn::ConstPtr &msg){
    button_loiter = Button_press_loiter.ButtonActive(msg);
    std::string result_loiter = button_loiter ? "YES" : "NO";
    ROS_INFO_STREAM("LOITER BUTTON ACTIVE? " << result_loiter);
    button_vessel = Button_press_vessel.ButtonActive(msg); 
    std::string result_vessel = button_vessel ? "YES" : "NO";
    ROS_INFO_STREAM("VESSEL BUTTON ACTIVE? " << result_vessel);
}

int main(int argc, char **argv){
  ros::init(argc,argv,"nest_control");
  ros::NodeHandle n;
  ros::NodeHandle nh;

  drone_vel = nh.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 1);
  ros::Subscriber sub_zarco = n.subscribe("/fw_asv0/p3d_odom", 1, zarco_speed);

  follow_point = nh.advertise<geometry_msgs::Pose>("/fw_asv0/follow_point",1);

  ros::Subscriber sub_drone = n.subscribe("/mavros/local_position/odom",1,vessel_following);
  
  ros::Subscriber sub_realpos_NEST = n.subscribe("/mavros_nest/local_position/pose",1,NEST_loiter);
  ros::Subscriber sub_mode_NEST = n.subscribe("/mavros_nest/state",1,cb_state);
  
  ros::Subscriber sub_button = n.subscribe("/mavros_nest/rc/in",1,cb_button);
  
  
  Button_press_loiter.min_value = 1100;
  Button_press_loiter.max_value = 1920;
  Button_press_loiter.channel = 11;
  
  Button_press_vessel.min_value = 1100;
  Button_press_vessel.max_value = 1920;
  Button_press_vessel.channel = 10;
  
  
  pid_x.kp = kd_x;
  pid_x.ki = ki_x;
  pid_x.kd = kd_x;
  
  pid_x.kp = kd_x;
  pid_x.ki = ki_x;
  pid_x.kd = kd_x;



  ros::spin();

  return 0;
}
