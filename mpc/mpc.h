/*! \file mpc.h
 *  
 *  \brief Model Predictive Controller header file
 */

#ifndef MPC_H_
#define MPC_H_

#include <cppad/cppad.hpp>
#include <Eigen/Core>
#include <helpers.h>



// set time stamp and duration
size_t N = 20; // number of steps to consider
double dt = 100e-3; // 100 milliseconds

const double Lf = 2.67;

/** The solver takes all the state variables and actuators variables in a 
    single vector. Thus, we should stablish where one variable starts and
    another ends.
 */
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N - 1;
size_t a_start = delta_start + N - 1;

/** Reference states*/
double ref_cte = 0.;
double ref_epsi = 0.;
double ref_v = 15.0; //mts / H


// using namespace CppAD::AD;

class FG_eval {
public:
    typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;
    Eigen::VectorXd coeffs;
    FG_eval(Eigen::VectorXd coeffs){
        this->coeffs = coeffs;
    };
    
    void operator()(ADvector & fg, const ADvector & vars){
        /** fg is a vector of constraints, vars is a vector of constraints
         *  The cost is stored in the first element of g, any additions
         * to the cost should be added to f[0]
        **/
        fg[0] = 0.;
        
        // the part of the cost based on the reference state
        for (int i = 0; i < N; i++){
            fg[0] += CppAD::pow(vars[cte_start + i] - ref_cte, 2);
            fg[0] += CppAD::pow(vars[epsi_start + i] - ref_epsi, 2);
            fg[0] += CppAD::pow(vars[v_start + i] - ref_v, 2);
        }
        
        /*** Setup constraints
         *   We add 1 to each of the starting indices due to cost being 
         *   located at index 0 of fg         
         */
        fg[1 + x_start] = vars[x_start];
        fg[1 + y_start] = vars[y_start];
        fg[1 + psi_start] = vars[psi_start];
        fg[1 + v_start] = vars[v_start];
        fg[1 + cte_start] = vars[cte_start];
        fg[1 + epsi_start] = vars[epsi_start];
        
        // the rest of the constraints
        for (int i = 0; i < N - 1; i++){
            // the state at time t 
            CppAD::AD<double> x0 = vars[x_start + i];
            CppAD::AD<double> y0 = vars[y_start + i];
            CppAD::AD<double> psi0 = vars[psi_start + i];
            CppAD::AD<double> v0 = vars[v_start + i];
            CppAD::AD<double> cte0 = vars[cte_start + i];
            CppAD::AD<double> epsi0 = vars[epsi_start + i];
            
            // the state at time t+1
            CppAD::AD<double> x1 = vars[x_start + i + 1];
            CppAD::AD<double> y1 = vars[y_start + i + 1];
            CppAD::AD<double> psi1 = vars[psi_start + i + 1];
            CppAD::AD<double> v1 = vars[v_start + i + 1];
            CppAD::AD<double> cte1 = vars[cte_start + i + 1];
            CppAD::AD<double> epsi1 = vars[epsi_start + i + 1];
            
            //only consider the actuator at time t
            CppAD::AD<double> delta0 = vars[delta_start + i];
            CppAD::AD<double> a0 = vars[a_start + i];
            
            // evaluate the polynomial
            
            CppAD::AD<double> f0 = coeffs[0] + coeffs[1]*x0 
                                    + coeffs[2]*CppAD::pow(x0, 2)
                                    + coeffs[3]*CppAD::pow(x0, 3);
            
            CppAD::AD<double> psi_des = CppAD::atan(coeffs[1] + x0*coeffs[2]*2
                                    + coeffs[3]*3*CppAD::pow(x0,2));
            
            fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
            fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
            fg[2 + psi_start +i] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
            fg[2 + v_start + i] = v1 - (v0 + a0 * dt);
            fg[2 + cte_start + i] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
            fg[2 + epsi_start + i] = epsi1 - ((psi0 - psi_des) + v0 * delta0 / Lf * dt);
        }
//        assert(fg.size() == 3);
        
    }
};


#endif