#include <iostream>
#include <Eigen/Core>
#include <Eigen/QR>
#include <vector>
#include "helpers.h"
#include "model/model.h"
#include "mpc/mpc.h"


//const double Lf = 2.67;

using namespace std;
using namespace Eigen;

int main(int argc, char * argv[]){
    
    model myModel;
    MPC myMpc;
    Eigen::VectorXd result(2);
    Eigen::VectorXd state(6);
    vector<double> xs;
    vector<double> ys;

    myModel.state.x = 909.40;
    myModel.state.y = 1128.60;
    
    cout << "current x: " << myModel.state.x << " current y: " << myModel.state.y 
        << " current psi: " << myModel.state.psi << " current v: " << myModel.state.v << endl;
        //
    // cout << "--------------------------------" << endl;
    // for (int i = 0; i < 5; i++){
    //     myModel.move(0.0, 1.0, 0.1);
    //     cout << "x: " << myModel.state.x
    //          << "\ty: " << myModel.state.y
    //          << "\tpsi: " << myModel.state.psi
    //          << "\tv: " << myModel.state.v
    //          << "\tsteer: " << myModel.state.steer
    //          << "\tthrottle: " << myModel.state.throttle << endl;
    // }
    // cout << "--------------------------------" << endl;

    // cout << "current x: " << myModel.state.x << " current y: " << myModel.state.y 
    //     << " current psi: " << myModel.state.psi << " current v: " << myModel.state.v << endl;

    VectorXd xvals(100);
    VectorXd yvals(100);
    int order = 3;
    VectorXd coeffs(order + 1);
    //
    STATE currstate;

    for (int j = 0; j < 500; j++){
        // waypoints
        // xvals << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10;
        // yvals << 1, 2, 5, 10, 17, 26, 37, 50, 65, 82, 101;
        xvals << 909.48, 909.486, 909.5, 909.51, 909.522, 909.612, 909.69, 909.786, 909.9, 909.986, 910.182, 910.35, 
                910.472, 910.74, 910.961, 911.201, 911.459, 911.641, 912.131, 912.34, 912.781, 913.131, 913.375, 
                913.885, 914.288, 914.707, 915.144, 915.445, 916.231, 916.724, 917.235, 917.763, 918.125, 918.87, 
                919.449, 919.845, 920.659, 921.289, 921.936, 922.6, 923.052, 923.977, 924.691, 925.176, 926.167, 
                926.931, 927.71, 928.506, 929.046, 930.145, 930.706, 931.274, 932.43, 933.316, 933.615, 934.829, 
                935.757, 936.701, 937.66, 937.984, 938.964, 940.293, 941.309, 942.339, 943.034, 943.736, 945.158, 
                945.879, 946.606, 947.708, 948.824, 949.576, 950.717, 951.869, 952.646, 953.823, 954.616, 955.414, 
                957.027, 957.842, 958.663, 959.077, 960.744, 962.009, 962.434, 963.714, 965.008, 965.441, 967.182, 
                968.054, 968.925, 969.795, 971.095, 972.394, 972.826, 974.556, 975.42, 976.287, 977.149, 978.443;
        yvals << 1128.67, 1128.67, 1128.67, 1128.67, 1128.67, 1128.67, 1128.67, 1128.67, 1128.67, 1128.67, 1128.67, 
                1128.67, 1128.67, 1128.67, 1128.67, 1128.67, 1128.67, 1128.67, 1128.67, 1128.67, 1128.67, 1128.67, 
                1128.68, 1128.68, 1128.68, 1128.69, 1128.69, 1128.69, 1128.7, 1128.71, 1128.72, 1128.73, 1128.74, 
                1128.77, 1128.79, 1128.8, 1128.82, 1128.84, 1128.86, 1128.88, 1128.9, 1128.93, 1128.96, 1128.98, 
                1129.01, 1129.04, 1129.07, 1129.11, 1129.13, 1129.17, 1129.19, 1129.22, 1129.27, 1129.32, 1129.33, 
                1129.4, 1129.45, 1129.51, 1129.56, 1129.59, 1129.65, 1129.74, 1129.82, 1129.9, 1129.95, 1130.01, 
                1130.14, 1130.21, 1130.27, 1130.38, 1130.5, 1130.58, 1130.71, 1130.84, 1130.93, 1131.08, 1131.17, 
                1131.28, 1131.48, 1131.59, 1131.7, 1131.75, 1131.98, 1132.16, 1132.22, 1132.4, 1132.59, 1132.66, 
                1132.93, 1133.07, 1133.2, 1133.35, 1133.57, 1133.79, 1133.87, 1134.19, 1134.35, 1134.53, 1134.7, 1134.97;
        // car state
    //    double px = 2.;
    //    double py = 0.;
    //    double psi = 0.;
    //    double v = 0.;
    //    double steer = 0.;
    //    double throttle = 0.;
        currstate = myModel.get_state();
        
        // convert from map to car frame
        frameConversion(currstate.x, currstate.y, currstate.psi, xvals, yvals);

        // print waypoints in car frame
//        cout << "xvals = [";
//        for (int i = 0; i < xvals.size(); i++)
//            cout << xvals[i] << ",";
//            cout << "]" << endl;
//        cout << "yvals = [";
//        for (int i = 0; i < yvals.size(); i++)
//            cout << yvals[i] << ",";
//            cout << "]" << endl;
//        // fit polynomial in car frame
        coeffs = polyfit(xvals, yvals, order);
        
        // print found coeffs
//        cout << "Polynomial Coefficients in car frame: ";
//        for (int i = 0; i < coeffs.size(); i++){
//            cout << coeffs[i] << ", ";
//        }
//        cout << endl;
//
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

        // cout << "Future state x: " << px
        //     << " y: " << py
        //     << " psi: " << psi 
        //     << " v: " << v << endl;
        // compute desired orientation equal to the atan of the derivative of
        // f(x) at point (px, py)
        // double psi_des = atan(coeffs[1] + currstate.x * coeffs[2] * 2 + coeffs[3] * 3 * pow(currstate.x, 2));
        double psi_des = atan(coeffs[1] + px * coeffs[2] * 2 + coeffs[3] * 3 * pow(px, 2));
        
        // compute error in orientation
        double epsi = 0 - psi_des + (v/Lf) * currstate.steer * currstate.throttle;
        
        // evaluate polynomial at (x, y)
        double cte = polyeval(coeffs, px) + v * sin(-psi_des);
        
        // cout << "Cross track error: " << cte << endl;
        // cout << "Orientation error: " << epsi << endl; 

        state << px, py, psi, v, cte, epsi;
        // solve miminization problem
        result = myMpc.solve(coeffs, state);

        // cout << result[0] << ", " << result[1] << endl;

        myModel.move(-result[0], result[1], dt);
        //currstate = myModel.get_state();

        // cout << "current x: " << myModel.state.x << " current y: " << myModel.state.y 
        //     << " current psi: " << myModel.state.psi << " current v: " << myModel.state.v 
        //     << " current steer: " << myModel.state.steer << " current throttle: " << myModel.state.throttle << endl;

//        currstate = myModel.get_state();
//        xs.push_back(currstate.x);
//        ys.push_back(currstate.y);
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
