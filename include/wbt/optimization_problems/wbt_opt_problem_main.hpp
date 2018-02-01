#ifndef WBT_OPTIMIZATION_PROBLEM_MAIN_H
#define WBT_OPTIMIZATION_PROBLEM_MAIN_H
#include <Utils/wrap_eigen.hpp>
#include <wbt/optimization_constants.hpp>
#include <string>

class Optimization_Problem_Main{
public:
  Optimization_Problem_Main(){}
  virtual ~Optimization_Problem_Main(){}

  std::string problem_name = "Undefined Optimization Problem";

  	virtual void get_init_opt_vars(std::vector<double> &x_vars){}
  	virtual void get_opt_vars_bounds(std::vector<double> &x_low, std::vector<double> &x_upp){}   	  	
  	virtual void update_opt_vars(std::vector<double> &x_vars){} 	  		

	virtual void get_F_bounds(std::vector<double> F_low, std::vector<double> F_upp){}
	virtual void get_F_obj_Row(int &obj_row){}

  	virtual void compute_F(std::vector<double> &F_eval){}
  	virtual void compute_F_constraints(std::vector<double> &F_eval){}
  	virtual void compute_F_objective_function(double &result_out){}

  	virtual void compute_G(std::vector<double> &G_eval, std::vector<int> &iGfun, std::vector<int> &jGvar, int &neG){}
	virtual void compute_A(std::vector<double> &A_eval, std::vector<int> &iAfun, std::vector<int> &jAvar, int &neA){}

};

#endif