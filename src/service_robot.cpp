#include <ros/ros.h>
#include "Service.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "service_robot") ;

  ros::NodeHandle nHandle ;
  
  Service service(nHandle) ;
  
  ros::spin();
  return 0;
}
