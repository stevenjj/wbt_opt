#include <wbt/wbt_snopt_wrapper.hpp>

namespace snopt_wrapper{
  Optimization_Problem_Main* ptr_optimization_problem;

  void wbt_FG(int    *Status, int *n,    double x[],
     int    *needF,  int *lenF,  double F[],
     int    *needG,  int *lenG,  double G[],
     char      *cu,  int *lencu,
     int    iu[],    int *leniu,
     double ru[],    int *lenru){

     }

  void solve_problem(Optimization_Problem_Main* input_ptr_optimization_problem){
  	std::cout << "[SNOPT Wrapper] Preparing Optimization Problem" << std::endl;
	ptr_optimization_problem = input_ptr_optimization_problem;
  	std::cout << "[SNOPT Wrapper] Problem Name: " << ptr_optimization_problem->problem_name << std::endl;

  }

}
