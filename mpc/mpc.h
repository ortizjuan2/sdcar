/*! \file mpc.h
 *  
 *  \brief Model Predictive Controller header file
 */

#ifndef MPC_H_
#define MPC_H_

#include <cppad/cppad.hpp>
#include <Eigen/Core>


// using namespace CppAD::AD;

class FG_eval {
public:
    typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;
    Eigen::VectorXd coeffs;
    FG_eval(Eigen::VectorXd coeffs){
        this->coeffs = coeffs;
    };
    
    void operator()(ADvector & fg, const ADvector & x){
//        assert(fg.size() == 3);
        CppAD::AD<double> x1 = x[0];
        
        //f(x) or cost function
        fg[0] = 0;
        //constraints
//        fg[1] = constraint....
//        // second constraint
//                fg[2] = constraint....
                return;
    }
};


#endif