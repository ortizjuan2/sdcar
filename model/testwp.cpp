/*
 * =====================================================================================
 *
 *       Filename:  testwp.cpp
 *
 *    Description:  test cases for wploader
 *
 *        Version:  1.0
 *        Created:  04/04/18 20:10:36
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  ortizjuan2@gmail.com
 *   Organization:  
 *
 * =====================================================================================
 */

#include <iostream>
#include "loadwp.h"

using namespace std;

int main(int argc, char * argv[]){

    if(argc != 2){
        cout << "Wrong number of parameters, please specify only file name of aypoints." << endl;
        exit(0);
    }

    string fileName = argv[1];
    cout << fileName << endl;

    waypointLoader wps(fileName.c_str());
    cout << "Number of waypoints loaded: " << wps.sz << endl;
    vector <WP> testwp = wps.getNextWp(10897);

    testwp = wps.getNextWp(6);

    for(int i = 0; i < testwp.size(); i++)
        cout << "x: " << testwp[i].x
            << " y: " << testwp[i].y
            << " z: " << testwp[i].z
            << " yaw: " << testwp[i].yaw
            << endl;



    return 0;
}

