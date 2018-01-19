#ifndef WBT_TIME_INTEGRATION_CONSTRAINT_H
#define WBT_TIME_INTEGRATION_CONSTRAINT_H

#include <Utils/wrap_eigen.hpp>
#include <string>
#include <iostream>

#include <wbt/hard_constraints/wbt_constraint_main.hpp>
#include "RobotModel.hpp"

class Time_Integration_Constraint: public Constraint_Function{
public:
	Time_Integration_Constraint();
	~Time_Integration_Constraint();
	RobotModel* robot_model_;	

	double dt; 
	int total_timesteps; 

	sejong::Vector q_vec;
	sejong::Vector qdot_vec;

	void derived_test_function();
};
#endif