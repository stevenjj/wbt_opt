#ifndef WBT_OPTIMIZATION_H
#define WBT_OPTIMIZATION_H

#include <stdio.h>
#include <string.h>
#include <iostream>

#include <Optimizer/snopt/include/snoptProblem.hpp>
#include "valkyrie_definition.h"
#include "RobotModel.hpp"

class WBT_Optimization{
public:
  static WBT_Optimization* GetWBT_Optimization();  
	RobotModel* robot_model_;

	void Initialization();
  void get_problem_functions();

	sejong::Vector m_q;
  sejong::Vector m_qdot;
  sejong::Vector m_tau;

  sejong::Vector cori_;
  sejong::Vector grav_;
  sejong::Matrix A_;
  sejong::Matrix Ainv_;	

	WBT_Optimization();
	~WBT_Optimization();	

};

#endif