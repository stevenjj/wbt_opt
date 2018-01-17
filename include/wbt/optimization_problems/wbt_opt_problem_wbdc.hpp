#ifndef WBT_OPTIMIZATION_PROBLEM_WBDC_H
#define WBT_OPTIMIZATION_PROBLEM_WBDC_H

#include <wbt/optimization_problems/wbt_opt_problem_main.hpp>

#include <wbt/containers/wbt_wholebody_task_list.hpp>
#include <wbt/containers/wbt_contact_list.hpp>

#include "valkyrie_definition.h"
#include "RobotModel.hpp"

class WBDC_Opt: public Optimization_Problem_Main{
public:
	WBDC_Opt();
	~WBDC_Opt();	

	void Initialization();
	void initialize_starting_configuration();
	void initialize_task_list();	
	void initialize_contact_list();

	RobotModel* robot_model;

	Contact_List contact_list;
	WholeBody_Task_List wb_task_list;

	sejong::Vector robot_q_init;
  	sejong::Vector robot_qdot_init; 
	
};

#endif