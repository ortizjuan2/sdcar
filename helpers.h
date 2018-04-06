/* 
 * File:   helpers.h
 * Author: jom
 *
 * Created on March 24, 2018, 4:52 PM
 */

#ifndef HELPERS_H
#define HELPERS_H

//#ifdef __cplusplus
//extern "C" {
//#endif

#include <Eigen/Core>
#include <Eigen/QR>
#include <vector>
    
/*! \fn double polyeval(Eigen::VectorXd coeffs, double x)
 * 
 * \brief polyeval helper function
 * 
 * Function used to evalate a polynomial.
 * \param [in] coeffs is a VectorXd with the coefficients of the polynomial
 * \param [in] x is a double for the base
 * \return the value of the polynomial
 */
 double polyeval(Eigen::VectorXd coeffs, double x);

 
 /*! \fn Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order)
  *  \brief polyfit helper function
  * 
  *  Fuction used to fit a polynomial. Adapted from
  *  https://github.com/JuliaMath?Polynomials.jl/blob/master/src/Polynomials.jl#676-L716
  *  \param [in] xvals a VectorXd with the x coordinates
  *  \param [in] yvals a VectorXd with the y coordinates
  *  \param [in] x is the order of the polynomial
  *  \return the coefficients of the polynomial
  */
 
 Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);

 
 /*! \fn void frameConversion(double px, double py, double psi, Eigen::VectorXd &ptx, Eigen::VectorXd &pty)
  *  \brief frame conversion helper function
  * 
  *  Function used to convert from global fram to car frame.
  *  \param [in] px is a double for the x coordinate
  *  \param [in] py is a double for the y coordinate
  *  \param [in] psi is the direction of the car
  *  \param [in, out] ptx is a VectorXd for the waypoints
  *  \param [in, out] pty is a VectorXd for the waypoints 
  */
 void frameConversion(double px, double py, double psi, Eigen::VectorXd &ptx, Eigen::VectorXd &pty);
 
//#ifdef __cplusplus
//}
//#endif

                

#endif /* HELPERS_H */

