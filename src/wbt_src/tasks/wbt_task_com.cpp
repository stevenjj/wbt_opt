#include <wbt/tasks/wbt_task_com.hpp>
#include "valkyrie_definition.h"

// Define COM Task ---------------------------------------------------------------
COM_Task::COM_Task(){
  robot_model_ = RobotModel::GetRobotModel();	
  task_dim = 3;
  task_name = "COM Task";
}
COM_Task::~COM_Task(){}
void COM_Task::getTaskJacobian(const sejong::Vector &q_state, sejong::Matrix & Jt){
	robot_model_->getCoMJacobian(q_state, Jt);
}
void COM_Task::getTaskJacobianDotQdot(const sejong::Vector &q_state, 
  							 		  const sejong::Vector &qdot_state, sejong::Vector & JtDotQdot){
	sejong::Matrix Ainv(NUM_QDOT, NUM_QDOT); Ainv.setZero();
	sejong::Vector b(NUM_QDOT); b.setZero();
    robot_model_->getInverseMassInertia(Ainv); 
    robot_model_->getCoriolis(b); 

	sejong::Matrix J_com;
	getTaskJacobian(q_state, J_com);

	JtDotQdot = J_com*Ainv*b;
}