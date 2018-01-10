#ifndef WBT_TASK_RIGHTFOOT_HAND_H
#define WBT_TASK_RIGHTFOOT_HAND_H

#include <wbt/tasks/wbt_task_main.hpp>
#include "RobotModel.hpp"

class RightFoot_RightHand_Task: public Task{
public:
	RightFoot_RightHand_Task();
	~RightFoot_RightHand_Task();	
	RobotModel* robot_model_;

	void getTaskJacobian(const sejong::Vector &q_state, sejong::Matrix & Jt);
    void getTaskJacobianDotQdot(const sejong::Vector &q_state, 
  							  const sejong::Vector &qdot_state, sejong::Vector & JtDotQdot);	
};
#endif