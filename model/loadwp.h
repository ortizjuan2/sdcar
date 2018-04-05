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
        std::vector<WP> getNextWp(int n);
    private:
        std::vector<WP> waypoints;
};

/* implementation */
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

std::vector<WP> waypointLoader::getNextWp(int n){
    // check inf end of road
    if ((this->indx + n) > this->sz)
        n = this->sz - this->indx;
    std::vector<WP> waypoints;
    for(int i = this->indx; i < (this->indx+n); i++){
        waypoints.push_back(this->waypoints[i]);
    }
    this->indx += n;
    if(this->indx >= this->sz-1)
        this->indx = 0;
    return waypoints;
}

#endif

