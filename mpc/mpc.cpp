#include <iostream>
#include <mpc.h>
#include <Eigen/Core>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>


// set time stamp and duration
size_t N = 5; // number of steps to consider
double dt = 100e-3; // 100 milliseconds

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


int main() {
  std::cout << "Hello World!\n";
  Eigen::VectorXd coeffs(4);
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
  vars[x_start] = 0;
  vars[y_start] = 0;
  vars[psi_start] = 0;
  vars[cte_start] = 5;
  vars[epsi_start] = 0;
  
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
  constraints_lowerbounds[x_start] = 0.0;
  constraints_lowerbounds[y_start] = 0.0;
  constraints_lowerbounds[psi_start] = 0.0;
  constraints_lowerbounds[v_start] = 0.0;
  constraints_lowerbounds[cte_start] = 5.0;
  constraints_lowerbounds[epsi_start] = 0.0;
  
  constraints_upperbounds[x_start] = 0.0;
  constraints_upperbounds[y_start] = 0.0;
  constraints_upperbounds[psi_start] = 0.0;
  constraints_upperbounds[v_start] = 0.0;
  constraints_upperbounds[cte_start] = 5.0;
  constraints_upperbounds[epsi_start] = 0.0;
  
  // object that computes the objective and constraints
  FG_eval fg_eval(coeffs);
  
  // solver options
  std::string options;
  options += "Integer   print_level     0\n";
  options += "Sparse    true            forward\n";
  options += "Sparse    true            reverse\n";
  options += "Numeric   max_cpu_time    0.05\n";
  
  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;
  
  // solve
  CppAD::ipopt::solve<Dvector, FG_eval>(options, vars, vars_lowerbounds,
          vars_upperbounds, constraints_lowerbounds, constraints_upperbounds,
          fg_eval, solution);
  
  std::cout << "Result: " << (solution.status==CppAD::ipopt::solve_result<Dvector>::success) << std::endl;
  
}
