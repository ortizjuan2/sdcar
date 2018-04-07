#include <iostream>
#include <Eigen/Core>
#include <Eigen/QR>
#include <vector>
#include "helpers.h"
#include "model/model.h"
#include "mpc/mpc.h"
#include "model/loadwp.h"

#define NUM_WPS 20

using namespace std;
using namespace Eigen;

int main(int argc, char * argv[]){
    
    model myModel;
    MPC myMpc;
    waypointLoader wploader("../model/wp_yaw.txt");
    Eigen::VectorXd result(2);
    Eigen::VectorXd state(6);
    vector<double> xs;
    vector<double> ys;

    vector<vector<double> > vals;
    
    

    myModel.state.x = 909.40;
    myModel.state.y = 1128.60;
    
    cout << "current x: " << myModel.state.x << " current y: " << myModel.state.y 
        << " current psi: " << myModel.state.psi << " current v: " << myModel.state.v << endl;

    VectorXd xvals(NUM_WPS);
    VectorXd yvals(NUM_WPS);
    
    int order = 3;
    VectorXd coeffs(order + 1);
    //
    STATE currstate;

    for (int j = 0; j < 2000; j++){

        currstate = myModel.get_state();

        vals = wploader.getNextWp(currstate.x, currstate.y, currstate.psi, NUM_WPS);
        for(int i = 0; i < vals[0].size(); i++){
            xvals[i] = vals[0][i];
            yvals[i] = vals[1][i];
        }
        // convert from map to car frame
        frameConversion(currstate.x, currstate.y, currstate.psi, xvals, yvals);

        coeffs = polyfit(xvals, yvals, order);
        
        xs.push_back(currstate.x);
        ys.push_back(currstate.y);
        
        /**
         * Predict the state 100ms into the future
         * before sending it to the solver
        */
        double px = currstate.v * cos(0) * dt;
        double py = currstate.v * sin(0) * dt;
        double psi = (currstate.v / Lf) * currstate.steer * dt;
        double v = currstate.v + (currstate.throttle * dt);

        double psi_des = atan(coeffs[1]); // + px * coeffs[2] * 2 + coeffs[3] * 3 * pow(px, 2));
        
       
        if(psi_des > PI){
            psi_des = PI;
        } else if (psi_des < -PI){
            psi_des = -PI;
        }
        
         // compute error in orientation
        double epsi = 0 - psi_des + (v/Lf) * currstate.steer * currstate.throttle;


        // evaluate polynomial at (x, y)
        double cte = polyeval(coeffs, 0);// + v * sin(-psi_des);
        
        state << 0, 0, 0, currstate.v, cte, -psi_des;
        // solve miminization problem
        result = myMpc.solve(coeffs, state);

        myModel.move(result[0], result[1], dt);
    }

    cout << "xs = [";
    for (int i = 0; i < xs.size(); i++)
        cout << xs[i] << ",";
        cout << "]" << endl;
    cout << "ys = [";
    for (int i = 0; i < ys.size(); i++)
        cout << ys[i] << ",";
        cout << "]" << endl;
    //
    cout << "current x: " << myModel.state.x << " current y: " << myModel.state.y 
        << " current psi: " << myModel.state.psi << " current v: " << myModel.state.v << endl;
    //
    return 0;
    
}
