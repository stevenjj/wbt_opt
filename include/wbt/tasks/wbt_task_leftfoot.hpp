#ifndef WBT_TASK_LEFTFOOT_H
#define WBT_TASK_LEFTFOOT_H

#include <wbt/tasks/wbt_task_main.hpp>
#include "RobotModel.hpp"

class LeftFoot_Task: public Task{
public:
	LeftFoot_Task();
	~LeftFoot_Task();	
	RobotModel* robot_model_;

	void getTaskJacobian(const sejong::Vector &q_state, sejong::Matrix & Jt);
    void getTaskJacobianDotQdot(const sejong::Vector &q_state, 
  							  const sejong::Vector &qdot_state, sejong::Vector & JtDotQdot);
};

#endif