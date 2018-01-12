#ifndef WBT_TIME_INTEGRATION_CONSTRAINT_H
#define WBT_TIME_INTEGRATION_CONSTRAINT_H

#include <Utils/wrap_eigen.hpp>
#include <string>
#include <iostream>
class Time_Integration_Constraint{
public:
	Time_Integration_Constraint();
	~Time_Integration_Constraint();

	double dt; 
	int total_timesteps; 

	sejong::Vector q_vec;
	sejong::Vector qdot_vec;
};
#endif