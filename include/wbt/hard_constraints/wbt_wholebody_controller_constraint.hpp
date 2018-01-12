#ifndef WBT_WHOLEBODY_CONTROLLER_CONSTRAINT_H
#define WBT_WHOLEBODY_CONTROLLER_CONSTRAINT_H

#include <Utils/wrap_eigen.hpp>
#include <string>
#include <iostream>
class Wholebody_Controller_Constraint{
public:
	Wholebody_Controller_Constraint();
	~Wholebody_Controller_Constraint();

	WholeBody_Task_List wb_task_list;
	sejong::Matrix B;
	sejong::Vector c;

	int task_dim;
};
#endif