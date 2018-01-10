#ifndef WBT_TASK_COM_H
#define WBT_TASK_COM_H

#include <wbt/tasks/wbt_task_main.hpp>
#include "RobotModel.hpp"

class COM_Task: public Task{
public:
	COM_Task();
	~COM_Task();	
	RobotModel* robot_model_;

    void getTaskJacobian(const sejong::Vector &q_state, sejong::Matrix & Jt);
    void getTaskJacobianDotQdot(const sejong::Vector &q_state, 
  							  const sejong::Vector &qdot_state, sejong::Vector & JtDotQdot);
};
#endif