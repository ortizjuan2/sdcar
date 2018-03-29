#include <iostream>
#include <Eigen/Core>
#include <Eigen/QR>
#include <vector>
#include "helpers.h"
#include "model/model.h"

const double Lf = 2.67;

using namespace std;
using namespace Eigen;

int main(int argc, char * argv[]){
    
    model myModel;
    
    cout << "current x: " << myModel.state.x << " current y: " << myModel.state.y 
        << " current psi: " << myModel.state.psi << " current v: " << myModel.state.v << endl;
        //
    cout << "--------------------------------" << endl;
    for (int i = 0; i < 5; i++){
        myModel.move(0.0, 1.0, 0.1);
        cout << "x: " << myModel.state.x
             << "\ty: " << myModel.state.y
             << "\tpsi: " << myModel.state.psi
             << "\tv: " << myModel.state.v
             << "\tsteer: " << myModel.state.steer
             << "\tthrottle: " << myModel.state.throttle << endl;
    }
    cout << "--------------------------------" << endl;

    cout << "current x: " << myModel.state.x << " current y: " << myModel.state.y 
        << " current psi: " << myModel.state.psi << " current v: " << myModel.state.v << endl;

    VectorXd xvals(11);
    VectorXd yvals(11);
    int order = 3;
    VectorXd coeffs(order + 1);
    //
    // waypoints
    xvals << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10;
    yvals << 1, 2, 5, 10, 17, 26, 37, 50, 65, 82, 101;
    
    // car state
//    double px = 2.;
//    double py = 0.;
//    double psi = 0.;
//    double v = 0.;
//    double steer = 0.;
//    double throttle = 0.;
    STATE currstate = myModel.get_state();
    
    // convert from map to car frame
    frameConversion(currstate.x, currstate.y, currstate.psi, xvals, yvals);

    // print waypoints in car frame
    cout << "xvals = [";
    for (int i = 0; i < xvals.size(); i++)
        cout << xvals[i] << ",";
        cout << "]" << endl;
    cout << "yvals = [";
    for (int i = 0; i < yvals.size(); i++)
        cout << yvals[i] << ",";
        cout << "]" << endl;
    // fit polynomial in car frame
    coeffs = polyfit(xvals, yvals, order);
    
    // print found coeffs
    cout << "Polynomial Coefficients in car frame: ";
    for (int i = 0; i < coeffs.size(); i++){
        cout << coeffs[i] << " ";
    }
    cout << endl;
    
    /**
     * Predict the state 100ms into the future
     * before sending it to the solver
    */
    double px = currstate.v * cos(0) * 100e-3;
    double py = currstate.v * sin(0) * 100e-3;
    double psi = (currstate.v / Lf) * currstate.steer * 100e-3;
    double v = currstate.v + (currstate.throttle * 100e-3);

    cout << "Future state x: " << px
        << " y: " << py
        << " psi: " << psi 
        << " v: " << v << endl;
    // compute desired orientation equal to the atan of the derivative of
    // f(x) at point (px, py)
    // double psi_des = atan(coeffs[1] + currstate.x * coeffs[2] * 2 + coeffs[3] * 3 * pow(currstate.x, 2));
    double psi_des = atan(coeffs[1] + px * coeffs[2] * 2 + coeffs[3] * 3 * pow(px, 2));
    
    // compute error in orientation
    double epsi = 0 - psi_des + (v/Lf) * currstate.steer * currstate.throttle;
    
    // evaluate polynomial at (x, y)
    double cte = polyeval(coeffs, px) + v * sin(-psi_des);
    
    cout << "Cross track error: " << cte << endl;
    cout << "Orientation error: " << epsi << endl; 
    
    
}
