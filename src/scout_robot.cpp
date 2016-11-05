#include <ros/ros.h>
#include "Scout.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scout_robot") ;

  ros::NodeHandle nHandle ;
  
  Scout scout(nHandle) ;
  
  ros::spin();
  return 0;
}
