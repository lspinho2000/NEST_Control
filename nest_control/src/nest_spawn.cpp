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
#include <gazebo_msgs/SetModelState.h>
#include <tf2/LinearMath/Quaternion.h>

#include<iostream>
#include<fstream>
#include<sstream>
#include<string>
#include <sstream>

//#include "pid.h"
//#include "nest_func.hpp"

ros::ServiceClient client;
gazebo_msgs::SetModelState srv;


std::string model_path = "~/catkin_ws/src/tools/Simulator/zarco_sim/urdf";

void drone_cb(const nav_msgs::OdometryConstPtr &msg){
  ros::Rate rate(30);
  srv.request.model_state.model_name = "nest";
  srv.request.model_state.reference_frame ="iris::base_link";

  //srv.request.model_state.pose.orientation = msg->pose.pose.orientation;
  srv.request.model_state.pose.position.x = 0;
  srv.request.model_state.pose.position.y = 0;
  srv.request.model_state.pose.position.z = 0;


  while (ros::ok()){
    if (client.call(srv))
      {
        ROS_INFO_STREAM("Spawning Model ==> " << srv.response.success);
      }
      else
      {
        ROS_ERROR("Failed to spawn model");
      }
    rate.sleep();
  }



}


int main(int argc, char **argv){
  ros::init(argc,argv,"nest_spawn");
  ros::NodeHandle n;
  ros::NodeHandle nh;

  client = n.serviceClient<const gazebo_msgs::SetModelState>("/gazebo/set_model_state");

  ros::Subscriber sub_drone = n.subscribe("/mavros/local_position/odom",1,drone_cb);

  ros::spin();
  return 0;
}
