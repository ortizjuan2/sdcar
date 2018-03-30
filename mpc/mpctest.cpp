#include <iostream>
#include "mpc.h"
#include <Eigen/Core>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>



int main() {
  
  Eigen::VectorXd coeffs(4);
  coeffs << 1.01 , 0.2 , 1.0 , 1.84136e-16;
  
  std::cout << coeffs << std::endl;


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
  vars[x_start] = 0.05;
  vars[y_start] = 0.0;
  vars[psi_start] = 0.0;
  vars[v_start] = 0.6;
  vars[cte_start] = 0.850091;
  vars[epsi_start] = -0.291457;
  
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
  //options += "Integer   print_level     0\n";
  //options += "String    sb              yes\n";
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
  
  std::cout << "Result: " << (solution.status==CppAD::ipopt::solve_result<Dvector>::success) << std::endl;
  std::cout << "Solution size: " << solution.x.size() << std::endl;
  std::cout << "#\tX\tY\tPSI\tV\tCTE\tEPSI" << std::endl;
  for (int i = 0; i < n_constraints/6.; i++){
      std::cout << i << "\t" << solution.x[i] 
                    << "\t" << solution.x[i+1]
                    << "\t" << solution.x[i+2]
                    << "\t" << solution.x[i+3]
                    << "\t" << solution.x[i+4]
                    << "\t" << solution.x[i+5] << std::endl;
  }

  std::cout << "solx = [";
  for(int i = 0; i < N; i++)
    std::cout << solution.x[i] << ",";
  std::cout << "]" << std::endl;
//
  std::cout << "soly = [";
  for(int i = 0; i < N; i++)
    std::cout << solution.x[y_start + i] << ",";
  std::cout << "]" << std::endl;

    std::cout << "Solution steer: " << solution.x[delta_start] << std::endl;
    std::cout << "Solution throttle: " << solution.x[a_start] << std::endl;

}
