#include <ros/ros.h>
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/ColorRGBA.h"
#include "scout_service/RoverPosition.h"
#include "scout_service/RoverList.h"
#include "scout_service/POIVector.h"
#include <vector>

typedef unsigned int UINT ;
using std::vector ;

class RoverListManager{
  public:
    RoverListManager(ros::NodeHandle) ;
    ~RoverListManager() {}
  private:
    ros::Subscriber subScoutA ; // hard-coded for pioneer1
    ros::Subscriber subServiceB ; // hard-coded for pioneer2
    ros::Subscriber subServiceC ; // hard-coded for pioneer3
    ros::Subscriber subScoutAPOI ; // hard-coded for pioneer1
    ros::Subscriber subServiceBPOI ; // hard-coded for pioneer2
    ros::Subscriber subServiceCPOI ; // hard-coded for pioneer2
    ros::Publisher pubScoutList ;
    ros::Publisher pubServiceList ;
    ros::Publisher pubPOIList ;
    ros::Publisher pubPOIMarkers ;
    ros::Publisher pubScoutPOIMarkers ;
    ros::Publisher pubServicePOIMarkers ;
    
    void scoutACallback(const scout_service::RoverPosition&) ;
    void serviceBCallback(const scout_service::RoverPosition&) ;
    void serviceCCallback(const scout_service::RoverPosition&) ;
    void POICallback(const scout_service::POIVector&) ;
    void servicePOICallback(const scout_service::POIVector&) ;
    
    scout_service::RoverList scouts ;
    scout_service::RoverList services ;
    scout_service::POIVector POIList ;
    scout_service::POIVector servicePOIList ;
    visualization_msgs::Marker allPOIs ;
    visualization_msgs::Marker scoutPOIs ;
    visualization_msgs::Marker servicePOIs ;
    
    void CollateScoutRovers(int, const scout_service::RoverPosition&) ;
    void CollateServiceRovers(int, const scout_service::RoverPosition&) ;
} ;

RoverListManager::RoverListManager(ros::NodeHandle nh){
  ROS_INFO("Initialising rover state manager...") ;
  subScoutA = nh.subscribe("/pioneer1/scout_position", 10, &RoverListManager::scoutACallback, this) ;
  subServiceB = nh.subscribe("/pioneer2/scout_position", 10, &RoverListManager::serviceBCallback, this) ;
  subServiceC = nh.subscribe("/pioneer3/scout_position", 10, &RoverListManager::serviceCCallback, this) ;
  subScoutAPOI = nh.subscribe("/pioneer1/discovered_POIs", 10, &RoverListManager::POICallback, this) ;
  subServiceBPOI = nh.subscribe("/pioneer2/serviced_POIs", 10, &RoverListManager::servicePOICallback, this) ;
  subServiceCPOI = nh.subscribe("/pioneer3/serviced_POIs", 10, &RoverListManager::servicePOICallback, this) ;
  pubScoutList = nh.advertise<scout_service::RoverList>("/scout_list", 10, true) ;
  pubServiceList = nh.advertise<scout_service::RoverList>("/service_list", 10, true) ;
  pubPOIList = nh.advertise<scout_service::POIVector>("/known_POIs", 10, true) ;
  pubPOIMarkers = nh.advertise<visualization_msgs::Marker>("/all_POIs", 10, true) ;
  pubScoutPOIMarkers = nh.advertise<visualization_msgs::Marker>("/scout_POIs", 10, true) ;
  pubServicePOIMarkers = nh.advertise<visualization_msgs::Marker>("/service_POIs", 10, true) ;
  POIList.num_pois = 0 ;
  pubPOIList.publish(POIList) ;
  
  double x ;
  double y ;
  scouts.rover_type = "scout" ;
  scouts.rover_names.push_back("pioneer1") ;
  ros::param::get("/pioneer1/x",x) ;
  ros::param::get("/pioneer1/y",y) ;
  scouts.x.push_back(x) ;
  scouts.y.push_back(y) ;
  
  services.rover_type = "service" ;
  services.rover_names.push_back("pioneer2") ;
  ros::param::get("/pioneer2/x",x) ;
  ros::param::get("/pioneer2/y",y) ;
  services.x.push_back(x) ;
  services.y.push_back(y) ;
  services.rover_names.push_back("pioneer3") ;
  ros::param::get("/pioneer3/x",x) ;
  ros::param::get("/pioneer3/y",y) ;
  services.x.push_back(x) ;
  services.y.push_back(y) ;
  
  allPOIs.header.seq = 0 ;
  allPOIs.header.stamp = ros::Time::now() ;
  allPOIs.header.frame_id = "/map" ;
  allPOIs.type = 7 ;
  allPOIs.scale.x = 0.2 ;
  allPOIs.scale.y = 0.2 ;
  allPOIs.scale.z = 0.2 ;
  allPOIs.color.r = 0.0 ;
  allPOIs.color.g = 1.0 ;
  allPOIs.color.b = 0.0 ;
  allPOIs.color.a = 1.0 ;
  
  vector<double> poiX ;
  vector<double> poiY ;
  ros::param::get("/poi_locations/x",poiX) ;
  ros::param::get("/poi_locations/y",poiY) ;
  
  for (UINT i = 0; i < poiX.size(); i++){
    geometry_msgs::Point p ;
    p.x = poiX[i] ;
    p.y = poiY[i] ;
    p.z = 0.0 ;
    allPOIs.points.push_back(p) ;
  }
  
  pubPOIMarkers.publish(allPOIs) ;
  
  scoutPOIs.header.seq = 0 ;
  scoutPOIs.header.stamp = ros::Time::now() ;
  scoutPOIs.header.frame_id = "/map" ;
  scoutPOIs.type = 7 ;
  scoutPOIs.scale.x = 0.5 ;
  scoutPOIs.scale.y = 0.5 ;
  scoutPOIs.scale.z = 0.5 ;
  scoutPOIs.color.r = 1.0 ;
  scoutPOIs.color.g = 0.0 ;
  scoutPOIs.color.b = 0.0 ;
  scoutPOIs.color.a = 1.0 ;
  
  servicePOIs.header.seq = 0 ;
  servicePOIs.header.stamp = ros::Time::now() ;
  servicePOIs.header.frame_id = "/map" ;
  servicePOIs.type = 7 ;
  servicePOIs.scale.x = 0.8 ;
  servicePOIs.scale.y = 0.8 ;
  servicePOIs.scale.z = 0.8 ;
  servicePOIs.color.r = 0.0 ;
  servicePOIs.color.g = 0.0 ;
  servicePOIs.color.b = 1.0 ;
  servicePOIs.color.a = 1.0 ;
  
  pubScoutList.publish(scouts) ;
  pubServiceList.publish(services) ;
  
  ROS_INFO("***** Rover state manager initialisation complete! *****") ;
}

void RoverListManager::scoutACallback(const scout_service::RoverPosition& msg){
  CollateScoutRovers(0, msg) ;
}

void RoverListManager::serviceBCallback(const scout_service::RoverPosition& msg){
  CollateServiceRovers(0, msg) ;
}

void RoverListManager::serviceCCallback(const scout_service::RoverPosition& msg){
  CollateServiceRovers(1, msg) ;
}

void RoverListManager::POICallback(const scout_service::POIVector& msg){
  bool repubPOIs = false ;
  for (UINT i = 0; i < msg.num_pois; i++){
    bool newPOI = true ;
    for (UINT j = 0; j < POIList.num_pois; j++){
      if (msg.x[i] == POIList.x[j] && msg.y[i] == POIList.y[j]){
        newPOI = false ;
        break ;
      }
    }
    if (newPOI){
      POIList.num_pois++ ;
      POIList.x.push_back(msg.x[i]) ;
      POIList.y.push_back(msg.y[i]) ;
      repubPOIs = true ;
      
      geometry_msgs::Point p ;
      p.x = msg.x[i] ;
      p.y = msg.y[i] ;
      p.z = 0.0 ;
      scoutPOIs.points.push_back(p) ;
    }
  }
  if (repubPOIs){
    pubPOIList.publish(POIList) ;
    pubScoutPOIMarkers.publish(scoutPOIs) ;
  }
}

void RoverListManager::servicePOICallback(const scout_service::POIVector& msg){
  bool repubPOIs = false ;
  for (UINT i = 0; i < msg.num_pois; i++){
    bool newPOI = true ;
    for (UINT j = 0; j < servicePOIList.num_pois; j++){
      if (msg.x[i] == servicePOIList.x[j] && msg.y[i] == servicePOIList.y[j]){
        newPOI = false ;
        break ;
      }
    }
    if (newPOI){
      servicePOIList.num_pois++ ;
      servicePOIList.x.push_back(msg.x[i]) ;
      servicePOIList.y.push_back(msg.y[i]) ;
      repubPOIs = true ;
      
      geometry_msgs::Point p ;
      p.x = msg.x[i] ;
      p.y = msg.y[i] ;
      p.z = 0.0 ;
      servicePOIs.points.push_back(p) ;
    }
  }
  if (repubPOIs)
    pubServicePOIMarkers.publish(servicePOIs) ;
}

void RoverListManager::CollateScoutRovers(int n, const scout_service::RoverPosition& msg){
  scouts.x[n] = msg.x ;
  scouts.y[n] = msg.y ;
  pubScoutList.publish(scouts) ;
}

void RoverListManager::CollateServiceRovers(int n, const scout_service::RoverPosition& msg){
  services.x[n] = msg.x ;
  services.y[n] = msg.y ;
  pubServiceList.publish(services) ;
}
