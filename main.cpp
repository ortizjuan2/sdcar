#include <iostream>
#include <Eigen/Core>
#include <Eigen/QR>
#include <vector>
#include "helpers.h"

const double Lf = 2.67;

using namespace std;
using namespace Eigen;

int main(int argc, char * argv[]){
    VectorXd xvals(11);
    VectorXd yvals(11);
    int order = 3;
    VectorXd coeffs(order + 1);
    
    xvals << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10;
    yvals << 1, 2, 5, 10, 17, 26, 37, 50, 65, 82, 101;
    
    // car state
    double px = 2.;
    double py = 0.;
    double psi = 0.;
    double v = 0.;
    double steer = 0.;
    double throttle = 0.;
    
    // convert from map to car frame
    frameConversion(px, py, psi, xvals, yvals);
    
    // fit polynomial in car frame
    coeffs = polyfit(xvals, yvals, order);
    
    
    // compute desired orientation equal to the atan of the derivative of
    // f(x) at point (px, py)
    double psi_des = atan(coeffs[1] + px * coeffs[2] * 2 + coeffs[3] * 3 * pow(px, 2));
    
    // compute error in orientation
    double epsi = 0 - psi_des + (v/Lf) * steer * throttle;
    
    // evaluate polynomial at (x, y)
    double cte = polyeval(coeffs, 0) + v * sin(-psi_des);
    
    cout << "Cross track error: " << cte << endl;
    
    
    
    
}
