#ifndef WBT_OPTIMIZATION_PROBLEM_WBDC_H
#define WBT_OPTIMIZATION_PROBLEM_WBDC_H

#include <wbt/optimization_problems/wbt_opt_problem_main.hpp>

#include <wbt/containers/wbt_opt_variable_list.hpp>
#include <wbt/containers/wbt_wholebody_task_list.hpp>
#include <wbt/containers/wbt_contact_list.hpp>
#include <wbt/containers/wbt_constraint_list.hpp>

#include <wbt/hard_constraints/wbt_wholebody_controller_constraint.hpp>
//#include <wbt/objective_functions/objective_function_main.hpp>
#include <wbt/objective_functions/objective_function_wbc_simple.hpp>

#include "valkyrie_definition.h"
#include "RobotModel.hpp"

class WBDC_Opt: public Optimization_Problem_Main{
public:
	WBDC_Opt();
	~WBDC_Opt();	

	RobotModel* 								robot_model;

	WBT_Opt_Variable_List 						opt_var_list;
	Constraint_List 							td_constraint_list; // Time Dependent Constraint List, exists for all timesteps
	Constraint_List 							ti_constraint_list;	// Time Independent Constraint List, exists for at a particular timestep

	WholeBody_Task_List 						wb_task_list;
	Contact_List 								contact_list;

	sejong::Vector 								robot_q_init;
  	sejong::Vector 								robot_qdot_init; 

  	int 										total_timesteps;


 	Wholebody_Controller_Constraint*  			ptr_wbc_constraint;

 	WBC_Objective_Function						objective_function;

  	
  	int constraint_size; // Unused

  	void get_init_opt_vars(std::vector<double> &x_vars);   	
  	void get_opt_vars_bounds(std::vector<double> &x_low, std::vector<double> &x_upp);   	  	
  	void update_opt_vars(std::vector<double> &x_vars); 	  		

	void get_F_bounds(std::vector<double> F_low, std::vector<double> F_upp);
	void get_F_obj_Row(int &obj_row);	

  	void compute_F(std::vector<double> &F_eval);
  	void compute_F_constraints(std::vector<double> &F_eval);
  	void compute_F_objective_function(double &result_out);


  	void compute_G(std::vector<double> &G_eval, std::vector<int> &iGfun, std::vector<int> &jGvar, int &neG);
	void compute_A(std::vector<double> &A_eval, std::vector<int> &iAfun, std::vector<int> &jAvar, int &neA);

private:
	void Initialization();
	void initialize_starting_configuration();
	void initialize_task_list();	
	void initialize_contact_list();
	void initialize_td_constraint_list();

	void initialize_opt_vars();

	void initialize_objective_func();


};

#endif