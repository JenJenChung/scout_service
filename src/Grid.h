#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <math.h>

using std::vector ;
using std::string ;
using std::stringstream ;
using std::ifstream ;
using std::getline ;

class Grid
{
  public:
    Grid() ;
    ~Grid(){}
    
    bool IsPOIDetected(double, double) ; // true if POI has previously been discovered, false otherwise
    void UpdateCell(double, double) ; // updates map with discoverec POI location
  private:
    int numRows ;
    int numCols ;
    int rOffset ;
    int cOffset ;
    double res ;
    vector< vector<int> > cells ; // first dimension maps to x, second dimension maps to y
    
    void WorldToCells(double, double, int&, int&) ; // convert odometry position to cell index
} ;

Grid::Grid(){
  // Read in parameters
  ros::param::get("cell_map/rows", numRows);
  ros::param::get("cell_map/columns", numCols);
  ros::param::get("cell_map/cell_offset/x", rOffset);
  ros::param::get("cell_map/cell_offset/y", cOffset);
  ros::param::get("cell_map/resolution", res);

  // Read in cell map
  char buffer[50] ;
  for (int i = 0; i < numRows; i++){
    vector<int> v ;
    sprintf(buffer,"cell_map/row%d",i) ;
    ros::param::get(buffer,v) ;
    cells.push_back(v) ;
  }

	ROS_INFO_STREAM("Cell map upload complete! " << numRows << " x " << numCols << " map at " << res << "m/cell") ;
	ROS_INFO_STREAM("Test cells.size(): " << cells.size() << ", cells[0][0]: " << cells[0][0]) ;
}

bool Grid::IsPOIDetected(double x, double y){
  int mx ;
  int my ;
  WorldToCells(x,y,mx,my) ;
  if (cells[mx][my] == 1)
    return true ;
  else
    return false ;
}

void Grid::UpdateCell(double x, double y){
  int mx ;
  int my ;
  WorldToCells(x,y,mx,my) ;
  cells[mx][my] = 1 ;
}

void Grid::WorldToCells(double x, double y, int& mx, int& my){
  mx = (int)round(x/res) ;
  my = (int)round(y/res) ;
  mx += rOffset ;
  my += cOffset ;
}
