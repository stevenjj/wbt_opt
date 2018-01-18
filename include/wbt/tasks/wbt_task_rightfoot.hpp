#ifndef WBT_TASK_RIGHTFOOT_H
#define WBT_TASK_RIGHTFOOT_H

#include <wbt/tasks/wbt_task_main.hpp>
#include "RobotModel.hpp"

class RightFoot_Task: public Task{
public:
	RightFoot_Task();
	~RightFoot_Task();	
	RobotModel* robot_model_;

	void getTaskJacobian(const sejong::Vector &q_state, sejong::Matrix & Jt);
    void getTaskJacobianDotQdot(const sejong::Vector &q_state, 
  							  const sejong::Vector &qdot_state, sejong::Vector & JtDotQdot);
};

#endif