/*! \file mpc.h
 *  
 *  \brief Model Predictive Controller header file
 */

#ifndef MPC_H
#define MPC_H

#include <cppad/cppad.hpp>
#include <Eigen/Core>
#include <helpers.h>
#include <cppad/ipopt/solve.hpp>



// set time stamp and duration
size_t N = 40; // number of steps to consider

double dt = 100e-3; // 100 milliseconds

// const double Lf = 2.67;

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
        

        // // Minimize the use of actuators
        // for (int i = 0; i < N-1; i++){
        //     fg[0] += 500 * CppAD::pow(vars[delta_start + i], 2); // steering
        //     fg[0] += 10 * CppAD::pow(vars[a_start + i], 2); // throttling
        // }

        // // Minimize the value gap between sequential actuations
        // for (int i = 0; i < N - 2; i++){
        //     fg[0] += 1000 * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
        //     fg[0] += 1000 * CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
        // }
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


class MPC{
    public:
    MPC();
    ~MPC();
    Eigen::VectorXd solve(Eigen::VectorXd coeffs, Eigen::VectorXd state);
};



MPC::MPC(){};
MPC::~MPC(){};


Eigen::VectorXd MPC::solve(Eigen::VectorXd coeffs, Eigen::VectorXd state) {
  
  //Eigen::VectorXd coeffs(4);
  Eigen::VectorXd result(2);
  // coeffs << 1.01 , 0.2 , 1.0 , 1.84136e-16;
  
//   std::cout << coeffs << std::endl;


  typedef CPPAD_TESTVECTOR(double) Dvector;
  
  /** number of independent variables including actuators **/
  size_t n_vars = N * 6 + (N - 1) * 2;
  
  /* set the number of constraints */
  size_t n_constraints = N * 6;

  // Initial value of the independent variables
  // should be zero except for the initial values.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++)
      vars[i] = 0.0;
  
  // set the initial variable values
  vars[x_start] = state[0];
  vars[y_start] = state[1];
  vars[psi_start] = state[2];
  vars[v_start] = state[3];
  vars[cte_start] = state[4];
  vars[epsi_start] = state[5];
  
  // lower and upper limits for the state variables
  Dvector vars_lowerbounds(n_vars);
  Dvector vars_upperbounds(n_vars);
  
  // set all non-actuator upper and lower limits
  // to the max negative and positive values.
  for (int i = 0; i < delta_start; i++){
      vars_lowerbounds[i] = -1.0e19;
      vars_upperbounds[i] = 1.0e19;
  }
  
  /** The upper and lower limits of delta are set to -25  and 25 in radians*/
  for (int i = delta_start; i < a_start; i++){
      vars_lowerbounds[i] = -0.436332;
      vars_upperbounds[i] = 0.436332;
  }
  
  /** acceleration and decceleration upper and lower limits, this can be 
   changed to something else taking into account the jerk**/
  for (int i = a_start; i < n_vars; i++){
      vars_lowerbounds[i] = -1.0;
      vars_upperbounds[i] = 1.0;
  }
  
  /** lower and upper limits for constraints, should be 0 except for the 
      initial values of the states at each start of index
   */
  Dvector constraints_lowerbounds(n_constraints);
  Dvector constraints_upperbounds(n_constraints);
  for (int i = 0; i < n_constraints; i++){
      constraints_lowerbounds[i] = 0.0;
      constraints_upperbounds[i] = 0.0;
  }
  constraints_lowerbounds[x_start] = vars[x_start];
  constraints_lowerbounds[y_start] = vars[y_start];
  constraints_lowerbounds[psi_start] = vars[psi_start];
  constraints_lowerbounds[v_start] = vars[v_start];
  constraints_lowerbounds[cte_start] = vars[cte_start];
  constraints_lowerbounds[epsi_start] = vars[epsi_start];
  
  constraints_upperbounds[x_start] = vars[x_start];
  constraints_upperbounds[y_start] = vars[y_start];
  constraints_upperbounds[psi_start] = vars[psi_start];
  constraints_upperbounds[v_start] = vars[v_start];
  constraints_upperbounds[cte_start] = vars[cte_start];
  constraints_upperbounds[epsi_start] = vars[epsi_start];
  
  // object that computes the objective and constraints
  FG_eval fg_eval(coeffs);
  
  // solver options
  std::string options;
  options += "Integer   print_level     0\n";
  options += "String    sb              yes\n";
  options += "Sparse    true            forward\n";
  options += "Sparse    true            reverse\n";
  //options += "Numeric   max_cpu_time    1.0\n";
  options += "Integer   max_iter        100\n";
  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;
  
  // solve
  CppAD::ipopt::solve<Dvector, FG_eval>(options, vars, vars_lowerbounds,
          vars_upperbounds, constraints_lowerbounds, constraints_upperbounds,
          fg_eval, solution);
  
//   std::cout << "Result: " << (solution.status==CppAD::ipopt::solve_result<Dvector>::success) << std::endl;
//   std::cout << "Solution size: " << solution.x.size() << std::endl;
//   std::cout << "#\tX\tY\tPSI\tV\tCTE\tEPSI" << std::endl;
//   for (int i = 0; i < n_constraints/6.; i++){
//       std::cout << i << "\t" << solution.x[i] 
//                     << "\t" << solution.x[i+1]
//                     << "\t" << solution.x[i+2]
//                     << "\t" << solution.x[i+3]
//                     << "\t" << solution.x[i+4]
//                     << "\t" << solution.x[i+5] << std::endl;
//   }

//   std::cout << "solx = [";
//   for(int i = 0; i < N; i++)
//     std::cout << solution.x[i] << ",";
//   std::cout << "]" << std::endl;
// //
//   std::cout << "soly = [";
//   for(int i = 0; i < N; i++)
//     std::cout << solution.x[y_start + i] << ",";
//   std::cout << "]" << std::endl;

//     std::cout << "Solution steer: " << solution.x[delta_start] << std::endl;
//     std::cout << "Solution throttle: " << solution.x[a_start] << std::endl;

    result << solution.x[delta_start], solution.x[a_start];

    return result;
}

#endif
