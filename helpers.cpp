/*! \file helpers.cpp 
 * \brief Implementation of the helper functions
 * Implementation of the helper functions used by main program
 */

#include <iostream>
#include <cmath>
#include <vector>
#include "helpers.h"



 double polyeval(Eigen::VectorXd coeffs, double x){
     double result = 0.;
     for (int i = 0; i < coeffs.size(); i++){
         result += coeffs[i] * pow(x, i);
     }
     return result;
 }
 
 
 
 Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order){
     assert(xvals.size() == yvals.size());
     assert(order >= 1 && order <= xvals.size() - 1);
     
     Eigen::MatrixXd A(xvals.size(), order + 1);
     
     for (int i = 0; i < xvals.size(); i++){
         A(i, 0) = 1.0;
     }
     
     for(int j = 0; j < xvals.size(); j++){
         for (int i = 0; i < order; i++){
             A(j, i + 1) = A(j, i) * xvals(j);
         }
     }
     
     auto Q = A.householderQr();
     auto result = Q.solve(yvals);
     return result;
 }  

 void frameConversion(double px, double py, double psi, Eigen::VectorXd &ptx, Eigen::VectorXd &pty){
     double cos_psi = cos(psi);
     double sin_psi = sin(psi);
     // rotation matrix to convert from map to car frame
     for (int i = 0; i < ptx.size(); i++){
         double x_tmp = (cos_psi * ptx[i]) + (sin_psi * pty[i]) - (cos_psi * px) - (sin_psi * py);
         double y_tmp = (-sin_psi * ptx[i]) + (cos_psi * pty[i]) + (sin_psi * px) - (cos_psi * py);
         
         ptx[i] = x_tmp;
         pty[i] = y_tmp;
     }
 }
  
