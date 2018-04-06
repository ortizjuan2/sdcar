/*
 * =====================================================================================
 *
 *       Filename:  loadwp.h
 *
 *    Description:  Load waypoints to simulate car road waypoints
 *
 *        Version:  1.0
 *        Created:  04/04/18 19:54:11
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  ortizjuan2@gmail.com 
 *   Organization:  
 *
 * =====================================================================================
 */
#ifndef LOADWP_H
#define LOADWP_H

#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <cstdlib>


#define PI 3.141592653589793


typedef struct {
    double x, y, z, yaw;
} WP;

class waypointLoader{
    public:
        std::string fileName;
        int indx;
        int sz;
        waypointLoader(std::string fileName);
        ~waypointLoader();
        std::vector<std::vector<double> > getNextWp(double px, double py,
                    double psi, int n);
        double euclidean_distance(double x1, double y1, double x2, double y2);
        int ClosestWaypoint(double x, double y);

        int NextWaypoint(double x, double y, double theta);    
    private:
        std::vector<WP> waypoints;
};




/* implementation */

double waypointLoader::euclidean_distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

int waypointLoader::ClosestWaypoint(double x, double y) {
  double closestLen = 100000;  // large number
  int closestWaypoint = 0;

  for (int i = 0; i < this->waypoints.size(); i++) {
    double map_x = this->waypoints[i].x;
    double map_y = this->waypoints[i].y;
    double dist = euclidean_distance(x, y, map_x, map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

int waypointLoader::NextWaypoint(double x, double y, double theta) {
  int closestWaypoint = ClosestWaypoint(x, y);

  double map_x = this->waypoints[closestWaypoint].x;
  double map_y = this->waypoints[closestWaypoint].y;

  double heading = atan2((map_y - y), (map_x - x));

  double angle = abs(theta - heading);

  if (angle > PI / 4) {
    closestWaypoint++;
  }

  return closestWaypoint;
}


waypointLoader::waypointLoader(std::string fileName){
    std::ifstream file(fileName.c_str());
    std::string line;
    double x,y,z,yaw;
    std::vector<WP> waypoints;
    WP wp;
    if(file.is_open()){
        while(getline(file, line)){
            std::istringstream iss(line);
            iss >> wp.x >> wp.y >> wp.z >> wp.yaw;
            waypoints.push_back(wp);
        }
        file.close();
    } else {
        std::cout << "Problem opening file " << fileName << std::endl;
        exit(0);
    }
    this->waypoints = waypoints;
    this->fileName = fileName;
    this->indx = 0;
    this->sz = waypoints.size();
}

/*  destructor */
waypointLoader::~waypointLoader(){};

std::vector<std::vector<double> > waypointLoader::getNextWp(double px,
                double py, double psi, int n){
    // check inf end of road

    int next = NextWaypoint(px, py, psi);
    if ((next + n) > this->sz)
        n = this->sz - next;
    std::vector<std::vector<double> > waypoints;
    std::vector<double> x;
    std::vector<double> y;
    for(int i = next; i < (next+n); i++){
//        waypoints.push_back(this->waypoints[i]);
        x.push_back(this->waypoints[i].x);
        y.push_back(this->waypoints[i].y);
    }
    waypoints.push_back(x);
    waypoints.push_back(y);
    this->indx += n;
    if(this->indx >= this->sz-1)
        this->indx = 0;
    return waypoints;
}

#endif

