#ifndef WBT_TASK
#define WBT_TASK
#include <Utils/wrap_eigen.hpp>
#include "RobotModel.hpp"

class Task{
public:
  Task(){}
  virtual ~Task(){}
  virtual void getTaskJacobian(const sejong::Vector &q_state, sejong::Matrix & Jt) = 0;
  virtual void getTaskJacobianDotQdot(const sejong::Vector &q_state, 
  							  		  const sejong::Vector &qdot_state, sejong::Vector & JtDotQdot) = 0;
  int task_dim;
};

class COM_Task: public Task{
public:
	COM_Task();
	~COM_Task();	

    void getTaskJacobian(const sejong::Vector &q_state, sejong::Matrix & Jt);
    void getTaskJacobianDotQdot(const sejong::Vector &q_state, 
  							  const sejong::Vector &qdot_state, sejong::Vector & JtDotQdot);
	RobotModel* robot_model_;
};

class LeftFootTask: public Task{
public:
	LeftFootTask();
	~LeftFootTask();	
	RobotModel* robot_model_;

	void getTaskJacobian(const sejong::Vector &q_state, sejong::Matrix & Jt);
    void getTaskJacobianDotQdot(const sejong::Vector &q_state, 
  							  const sejong::Vector &qdot_state, sejong::Vector & JtDotQdot);
};

class RightFoot_Hand_Task: public Task{
public:
	RightFoot_Hand_Task();
	~RightFoot_Hand_Task();	
	RobotModel* robot_model_;

	void getTaskJacobian(const sejong::Vector &q_state, sejong::Matrix & Jt);
    void getTaskJacobianDotQdot(const sejong::Vector &q_state, 
  							  const sejong::Vector &qdot_state, sejong::Vector & JtDotQdot);	
};

#endif