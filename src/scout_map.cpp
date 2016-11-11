#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <iostream>
#include "ScoutCostmap.h"
#include <ros/console.h>

int main(int argc, char** argv){

  ros::init(argc,argv,"scout_map");
  ros::NodeHandle nh ;

  tf::TransformListener tf_(ros::Duration(10));

  costmap_2d::Costmap2DROS *costmap_ros_ = new costmap_2d::Costmap2DROS("scout_map", tf_);
  costmap_ros_->start();
  
  ScoutCostmap scout_costmap(nh,costmap_ros_) ;
  
  ros::spin();
  
  delete costmap_ros_ ;
  costmap_ros_ = 0 ;
  
  return 0;
}
