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
    
    cout << "current x: " << myModel.state.x << " current y: " << myModel.state.y << endl;
    
    for (int i = 0; i < 20; i++){
        myModel.move(1.0, 1.0, 0.1);
        cout << "x: " << myModel.state.x
             << " y: " << myModel.state.y
             << " psi: " << myModel.state.psi
             << " v: " << myModel.state.v
             << " steer: " << myModel.state.steer
             << " throttle: " << myModel.state.throttle << endl;
    }
    VectorXd xvals(11);
    VectorXd yvals(11);
    int order = 3;
    VectorXd coeffs(order + 1);
    
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
    
    // fit polynomial in car frame
    coeffs = polyfit(xvals, yvals, order);
    
    // print found coeffs
    cout << "Polynomial Coefficients: ";
    for (int i = 0; i < coeffs.size(); i++){
        cout << coeffs[i] << " ";
    }
    cout << endl;
    
    // compute desired orientation equal to the atan of the derivative of
    // f(x) at point (px, py)
    double psi_des = atan(coeffs[1] + currstate.x * coeffs[2] * 2 + coeffs[3] * 3 * pow(currstate.x, 2));
    
    // compute error in orientation
    double epsi = 0 - psi_des + (currstate.v/Lf) * currstate.steer * currstate.throttle;
    
    // evaluate polynomial at (x, y)
    double cte = polyeval(coeffs, 0) + currstate.v * sin(-psi_des);
    
    cout << "Cross track error: " << cte << endl;
    
    
}
