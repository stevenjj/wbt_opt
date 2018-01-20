#ifndef WBT_WHOLEBODY_CONTROLLER_CONSTRAINT_H
#define WBT_WHOLEBODY_CONTROLLER_CONSTRAINT_H

#include <Utils/wrap_eigen.hpp>
#include <string>
#include <iostream>

#include <wbt/hard_constraints/wbt_constraint_main.hpp>
#include <wbt/containers/wbt_wholebody_task_list.hpp>
#include "RobotModel.hpp"

class Wholebody_Controller_Constraint: public Constraint_Function{
public:
	Wholebody_Controller_Constraint();
	Wholebody_Controller_Constraint(WholeBody_Task_List* wb_task_list_input);	
	~Wholebody_Controller_Constraint();
	RobotModel* robot_model;	

	WholeBody_Task_List* wb_task_list;
	sejong::Matrix B;
	sejong::Vector c;

	int task_dim;

	void set_task_list(WholeBody_Task_List* wb_task_list_input);

};
#endif