#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include "geometry_msgs/Point.h"
#include "scout_service/POIVector.h"
#include <ros/console.h>
#include <math.h>
#include <vector>

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
    ros::Publisher pubPOIAll ;
    
    costmap_2d::Costmap2DROS * costmap_ros_ ;
    UINT x_max ;
    UINT y_max ;
    scout_service::POIVector POIList ;
    unsigned char cellCostThreshold ;
    vector<int> poiX ;
    vector<int> poiY ;
    
    Grid POIMap ;
    
    void mapUpdateCallback(const map_msgs::OccupancyGridUpdate&) ;
} ;

ScoutCostmap::ScoutCostmap(ros::NodeHandle nh, costmap_2d::Costmap2DROS *cm_ros_): costmap_ros_(cm_ros_), cellCostThreshold(50){
  ROS_INFO("Initialising scout map object...") ;
  subCostmapUpdate = nh.subscribe("scout_map/scout_map/costmap_updates", 10, &ScoutCostmap::mapUpdateCallback, this) ;
  pubPOIList = nh.advertise<scout_service::POIVector>("discovered_POIs", 10, true) ;
  pubPOIAll = nh.advertise<geometry_msgs::Point>("current_POIs", 10, true) ;

  costmap_2d::Costmap2D *costmap_ = costmap_ros_->getCostmap() ;
  x_max = costmap_->getSizeInCellsX() ;
  y_max = costmap_->getSizeInCellsY() ;
  
  POIList.num_pois = 0 ;
  pubPOIList.publish(POIList) ;
  
  ros::param::get("/poi_locations/x",poiX) ;
  ros::param::get("/poi_locations/y",poiY) ;
  ROS_INFO("***** Scout map initialisation complete! *****") ;
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
        // Cross-reference with known POI locations to avoid labelling other robots as POIs
        bool isPOI = false ;
        for (UINT k = 0; k < poiX.size(); k++){
          if ((int)round(wx) == poiX[k] && (int)round(wy) == poiY[k]){
            isPOI = true ;
            break ;
          }
        }
        if (isPOI){
          geometry_msgs::Point p ;
          p.x = round(wx) ;
          p.y = round(wy) ;
          p.z = 0.0 ;
          pubPOIAll.publish(p) ;
        }
        if (isPOI && !POIMap.IsPOIDetected(wx,wy)){ // new POI detected
          POIMap.UpdateCell(wx,wy) ;
          POIList.num_pois++ ;
          POIList.x.push_back(round(wx)) ;
          POIList.y.push_back(round(wy)) ;
          repubPOIs = true ;
        }
      }
    }
  }
  
  if (repubPOIs)
    pubPOIList.publish(POIList) ; // announce POI list change
}
