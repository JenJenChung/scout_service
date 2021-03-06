#include <ros/ros.h>
#include <ros/console.h>
#include <string>
#include <vector>
#include "std_msgs/Int8.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "scout_service/RoverPosition.h"
#include "scout_service/RoverList.h"
#include "scout_service/RoverState.h"
#include "scout_service/POIVector.h"
#include <move_base_msgs/MoveBaseActionResult.h>
#include <math.h>

#define PI 3.14159265358979

using std::vector ;

typedef unsigned int UINT ;

class Service{
  public:
    Service(ros::NodeHandle) ;
    ~Service(){}
  private:
    ros::Subscriber subResult ;
    ros::Subscriber subAMCLPose ;
    ros::Subscriber subScout ;
    ros::Subscriber subService ;
    ros::Subscriber subPOIs ;
    ros::Subscriber subAction ;
    ros::Publisher pubServicePosition ;
    ros::Publisher pubFullState ;
    ros::Publisher pubWaypoint ;
    ros::Publisher pubServicedPOIs ;
    ros::Publisher pubAllPOIs ;
    
    scout_service::RoverPosition pos ;
    
    std::string rover_name ;
    vector<double> full_state ; // 48 dimensional control policy input
    vector<double> poi_states ; // control policy input states related to POIs
    vector<double> scout_states ; // control policy input states related to scout rovers
    vector<double> service_states ; // control policy input states related to service rovers
    bool isResult ; // true if move_base has returned an action result (e.g. waypoint reached)
    bool isScout ; // true if scout list has been updated since last waypoint was set
    bool isService ; // true if service list has been updated since last waypoint was set
    int totalPOIs ;
    int totalScouts ;
    int totalServices ;
    vector<double> poiX ;
    vector<double> poiY ;
    double worldSize ;
    
    UINT CalculateSector(double, double) ;
    double CalculateDistance(double, double) ;
    void SendFullState() ;
    void RecalculatePOIStates() ;
    void CheckIfServiced() ;
    
    void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped&) ;
    void scoutCallback(const scout_service::RoverList&) ;
    void serviceCallback(const scout_service::RoverList&) ;
    void poiCallback(const scout_service::POIVector&) ;
    void waypointCallback(const move_base_msgs::MoveBaseActionResult&) ;
    void actionCallback(const std_msgs::Int8&) ;
} ;

Service::Service(ros::NodeHandle nh){
  ROS_INFO("Initialising service parameters...") ;
  subResult = nh.subscribe("move_base/result", 10, &Service::waypointCallback, this) ;
  subAMCLPose = nh.subscribe("amcl_pose", 10, &Service::amclPoseCallback, this) ;
  subScout = nh.subscribe("/scout_list", 10, &Service::scoutCallback, this) ;
  subService = nh.subscribe("/service_list", 10, &Service::serviceCallback, this) ;
  subPOIs = nh.subscribe("/known_POIs", 10, &Service::poiCallback, this) ;
  subAction = nh.subscribe("action", 10, &Service::actionCallback, this) ;
  pubServicePosition = nh.advertise<scout_service::RoverPosition>("scout_position", 10) ;
  pubFullState = nh.advertise<scout_service::RoverState>("full_state", 10) ;
  pubWaypoint = nh.advertise<geometry_msgs::Twist>("map_goal", 10) ;
  pubServicedPOIs = nh.advertise<scout_service::POIVector>("serviced_POIs", 10, true) ;
  pubAllPOIs = nh.advertise<geometry_msgs::Point>("current_POIs", 10, true) ;
  
  // Initialise from parameter list
  ros::param::get("rover_name",rover_name) ;
  pos.rover_name = rover_name ;
  ros::param::get("/total_scouts",totalScouts) ;
  ros::param::get("/total_services",totalServices) ;
  ros::param::get("/world_size",worldSize) ; // normalisation factor for distances
  ros::param::get("/total_POIs",totalPOIs) ;
  poiX.clear() ;
  poiY.clear() ;

  isResult = false ;
  isScout = false ;
  isService = false ;
  
  ROS_INFO("***** Service initialisation complete! *****") ;
}

void Service::amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg){ // publish your own current position
  pos.header = msg.header ;
  pos.x = msg.pose.pose.position.x ;
  pos.y = msg.pose.pose.position.y ;
  pos.z = msg.pose.pose.position.z ;
  
  pubServicePosition.publish(pos) ;
  
  RecalculatePOIStates() ;
}

void Service::scoutCallback(const scout_service::RoverList& msg){
  // Reset scout states to 0
  vector<double> ss(16,0.0) ;
  scout_states = ss ;
  for (UINT i = 0; i < msg.rover_names.size(); i++){
    UINT s = CalculateSector(msg.x[i],msg.y[i]) ;
    double d = CalculateDistance(msg.x[i],msg.y[i]) ;
    scout_states[2*s] += 1.0 ; // increment total number of scouts in sector
    scout_states[2*s+1] += d ; // increment total distances to other scouts in sector
  }
  
  // Normalise states
  for (UINT i = 0; i < scout_states.size(); i++){
    if (i%2 == 0)
      scout_states[i] /= totalScouts ;
    else
      scout_states[i] /= worldSize ;
  }
  
  isScout = true ;
  SendFullState() ;
}

void Service::serviceCallback(const scout_service::RoverList& msg){
  // Reset service states to 0
  vector<double> ss(16,0.0) ;
  service_states = ss ;
  for (UINT i = 0; i < msg.rover_names.size(); i++){
    if (rover_name.compare(msg.rover_names[i]) != 0){ // different service to yourself
      UINT s = CalculateSector(msg.x[i],msg.y[i]) ;
      double d = CalculateDistance(msg.x[i],msg.y[i]) ;
      service_states[2*s] += 1.0 ; // increment total number of services in sector
      service_states[2*s+1] += d ; // increment total distances to other services in sector
    }
  }
  
  // Normalise states
  for (UINT i = 0; i < service_states.size(); i++){
    if (i%2 == 0)
      service_states[i] /= totalServices ;
    else
      service_states[i] /= worldSize ;
  }
  isService = true ;
  SendFullState() ;
}

void Service::poiCallback(const scout_service::POIVector& msg){
  poiX.clear() ;
  poiY.clear() ;
  
  for (UINT i = 0; i < msg.num_pois; i++){
    poiX.push_back(msg.x[i]) ;
    poiY.push_back(msg.y[i]) ;
  }
  
  RecalculatePOIStates() ;
}

void Service::waypointCallback(const move_base_msgs::MoveBaseActionResult& msg){
  isResult = true ;
  SendFullState() ;
}


void Service::SendFullState(){
  if (isScout && isService && isResult){
    // Concatenate control policy state
    full_state.clear() ;
    for (UINT i = 0; i < poi_states.size()/2; i++){
      full_state.push_back(poi_states[2*i]) ;
      full_state.push_back(poi_states[2*i+1]) ;
      full_state.push_back(scout_states[2*i]) ;
      full_state.push_back(scout_states[2*i+1]) ;
      full_state.push_back(service_states[2*i]) ;
      full_state.push_back(service_states[2*i+1]) ;
    }
    
    scout_service::RoverState state ;
    state.rover_name = rover_name ;
    state.data = full_state ;
    
    pubFullState.publish(state) ;

    isResult = false ;
//    isScout = false ; // only require first pose msg to be received
//    isService = false ; // only require first pose msg to be received
  }
}

void Service::actionCallback(const std_msgs::Int8& msg){
  // compute new waypoint from control policy
  geometry_msgs::Twist waypoint ;
  double x ;
  double y ;
  
  if (msg.data == 0){
    x = 0.0 ;
    y = 0.0 ;
  }
  else if (msg.data == 1){ // right
    x = 0.0 ;
    y = 1.0 ;
  }
  else if (msg.data == 2){ // down
    x = 1.0 ;
    y = 0.0 ;
  }
  else if (msg.data == 3){ // left
    x = 0.0 ;
    y = -1.0 ;
  }
  else if (msg.data == 4){ // up
    x = -1.0 ;
    y = 0.0 ;
  }
  
  // apply control policy here
  waypoint.linear.x = round(pos.x) + x ;
  waypoint.linear.y = round(pos.y) + y ;
  waypoint.linear.z = 0.0 ;
  waypoint.angular.x = 0.0 ;
  waypoint.angular.y = 0.0 ;
  waypoint.angular.z = 0.0 ;
  
  pubWaypoint.publish(waypoint) ;
}

UINT Service::CalculateSector(double x, double y){
  double diffy = y - pos.y ;
  double diffx = x - pos.x ;
  double a = atan2(diffy,diffx) * 180.0/PI ; // angle is from standard x axis definition
  
  // sectors counted from north vector
  UINT s = 0 ;
  if (a < -135.0)
    s = 5 ;
  else if (a < -90.0)
    s = 4 ;
  else if (a < -45.0)
    s = 3 ;
  else if (a < 0.0)
    s = 2 ;
  else if (a < 45.5)
    s = 1 ;
  else if (a < 90.0)
    s = 0 ;
  else if (a < 135.0)
    s = 7 ;
  else
    s = 6 ;
  
  return s ;
}

double Service::CalculateDistance(double x, double y){
  double diffy = y - pos.y ;
  double diffx = x - pos.x ;
  double dist = sqrt(diffx*diffx + diffy*diffy) ;
  return dist ;
}

void Service::RecalculatePOIStates(){
  // Recalculate POI state inputs
  vector<double> p(16,0.0) ;
  poi_states = p ;
  for (UINT i = 0; i < poiX.size(); i++){
    UINT s = CalculateSector(poiX[i],poiY[i]) ;
    double d = CalculateDistance(poiX[i],poiY[i]) ;
    poi_states[2*s] += 1.0 ; // increment total number of scouts in sector
    poi_states[2*s+1] += d ; // increment total distances to other scouts in sector
  }
  
  // Normalise states
  for (UINT i = 0; i < poi_states.size(); i++){
    if (i%2 == 0)
      poi_states[i] /= totalPOIs ;
    else
      poi_states[i] /= worldSize ;
  }
  
  CheckIfServiced() ;
}

void Service::CheckIfServiced(){
  bool poisServiced = false ;
  scout_service::POIVector sPOIs ;
  sPOIs.num_pois = 0 ;
  for (UINT i = 0; i < poiX.size(); i++){
    if (fabs(poiX[i]-pos.x) <= 1.5 && fabs(poiY[i]-pos.y) <= 1.5){
      ROS_INFO("Servicing a POI") ;
      poisServiced = true ;
      sPOIs.num_pois++ ;
      sPOIs.x.push_back(poiX[i]) ;
      sPOIs.y.push_back(poiY[i]) ;
      
      geometry_msgs::Point p ;
      p.x = poiX[i] ;
      p.y = poiY[i] ;
      p.z = 0.0 ;
      pubAllPOIs.publish(p) ;
    }
  }
  if (poisServiced)
    pubServicedPOIs.publish(sPOIs) ;
}
