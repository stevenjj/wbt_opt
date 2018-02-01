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
	Constraint_List 							constraint_list;

	WholeBody_Task_List 						wb_task_list;
	Contact_List 								contact_list;

	sejong::Vector 								robot_q_init;
  	sejong::Vector 								robot_qdot_init; 

  	int 										total_timesteps;


 	Wholebody_Controller_Constraint*  			ptr_wbc_constraint;

 	WBC_Objective_Function						objective_function;

  	void initialize_F_bounds();

  	void compute_F_objective_function();

  	int constraint_size;
  	void compute_F_constraints();

  	void compute_G(std::vector<double> &G_eval, std::vector<int> &iGfun, std::vector<int> &jGvar, int &neG);
	void compute_A(std::vector<double> &A, std::vector<int> &iAfun, std::vector<int> &jAvar, int &neA);

private:
	void Initialization();
	void initialize_starting_configuration();
	void initialize_task_list();	
	void initialize_contact_list();
	void initialize_constraint_list();

	void initialize_opt_vars();

	void initialize_objective_func();


};

#endif