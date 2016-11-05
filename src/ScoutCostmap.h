#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include "scout_service/POIVector.h"
#include <ros/console.h>

#include "Grid.h"

typedef unsigned int UINT ;

class ScoutCostmap
{
  public:
    ScoutCostmap(ros::NodeHandle, costmap_2d::Costmap2DROS *) ;
    ~ScoutCostmap() {}
    
  private:
    ros::Subscriber subCostmapUpdate ;
    ros::Publisher pubPOIList ;
    
    costmap_2d::Costmap2DROS * costmap_ros_ ;
    UINT x_max ;
    UINT y_max ;
    scout_service::POIVector POIList ;
    unsigned char cellCostThreshold ;
    
    Grid POIMap ;
    
    void mapUpdateCallback(const map_msgs::OccupancyGridUpdate&) ;
} ;

ScoutCostmap::ScoutCostmap(ros::NodeHandle nh, costmap_2d::Costmap2DROS *cm_ros_): costmap_ros_(cm_ros_), cellCostThreshold(50){
  ROS_INFO("Initialising scout field of view...") ;
  subCostmapUpdate = nh.subscribe("scout_map/costmap_updates", 10, &ScoutCostmap::mapUpdateCallback, this) ;
  pubPOIList = nh.advertise<scout_service::POIVector>("discovered_POIs", 10) ;

  costmap_2d::Costmap2D *costmap_ = costmap_ros_->getCostmap() ;
  x_max = costmap_->getSizeInCellsX() ;
  y_max = costmap_->getSizeInCellsY() ;
  
  POIList.num_pois = 0 ;
}

void ScoutCostmap::mapUpdateCallback(const map_msgs::OccupancyGridUpdate&){
  costmap_2d::Costmap2D *costmap_ = costmap_ros_->getCostmap() ;
  
  double wx ;
  double wy ;
  bool repubPOIs = false ;
  for (UINT i = 0; i < x_max; i++){
    for (UINT j = 0; j < y_max; j++){
      if (costmap_->getCost(i,j) > cellCostThreshold){
        costmap_->mapToWorld(i,j,wx,wy) ;
        if (!POIMap.IsPOIDetected(wx,wy)){ // new POI detected
          POIMap.UpdateCell(wx,wy) ;
          POIList.num_pois++ ;
          POIList.x.push_back(wx) ;
          POIList.y.push_back(wy) ;
          repubPOIs = true ;
        }
      }
    }
  }
  
  pubPOIList.publish(POIList) ; // announce POI list change
}
