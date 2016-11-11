#include <ros/ros.h>
#include "RoverListManager.h"
#include <ros/console.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "rover_list_manager") ;

  ros::NodeHandle nHandle ;
  
  RoverListManager lists(nHandle) ;
  
  ros::spin();
  return 0;
}
