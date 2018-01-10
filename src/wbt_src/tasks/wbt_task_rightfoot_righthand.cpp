#include <wbt/tasks/wbt_task_rightfoot_righthand.hpp>
#include "valkyrie_definition.h"

// Define RightFoot Task ---------------------------------------------------------------
RightFoot_RightHand_Task::RightFoot_RightHand_Task(){
	robot_model_ = RobotModel::GetRobotModel();
	task_dim = 7; // 4 for the right foot, and 3 for the right hand 	
}
RightFoot_RightHand_Task::~RightFoot_RightHand_Task(){}

void RightFoot_RightHand_Task::getTaskJacobian(const sejong::Vector &q_state, sejong::Matrix & Jt){
	sejong::Matrix Jrfoot_tmp;
	sejong::Matrix Jrhand_tmp;
    robot_model_->getFullJacobian(q_state, SJLinkID::LK_rightCOP_Frame, Jrfoot_tmp);
    robot_model_->getFullJacobian(q_state, SJLinkID::LK_rightForearmLink, Jrhand_tmp);   

	sejong::Matrix Jrfoot_task = Jrfoot_tmp.block(2, 0, 4, NUM_QDOT);
	sejong::Matrix Jrhand_task = Jrhand_tmp.block(3, 0, 3, NUM_QDOT);

	sejong::Matrix J_tmp(task_dim, NUM_QDOT); J_tmp.setZero();
	J_tmp.block(0, 0, 4, NUM_QDOT) = Jrfoot_task;
	J_tmp.block(4, 0, 3, NUM_QDOT) = Jrhand_task;

	Jt = J_tmp;
}

void RightFoot_RightHand_Task::getTaskJacobianDotQdot(const sejong::Vector &q_state, 
  							  			  const sejong::Vector &qdot_state, sejong::Vector & JtDotQdot){
	sejong::Matrix Jdot_rfoot_tmp;
	sejong::Matrix Jdot_rhand_tmp;

    robot_model_->getFullJacobianDot(q_state, qdot_state, SJLinkID::LK_rightCOP_Frame, Jdot_rfoot_tmp);
    robot_model_->getFullJacobianDot(q_state, qdot_state, SJLinkID::LK_rightForearmLink, Jdot_rhand_tmp);   

	sejong::Matrix Jdot_rfoot_task = Jdot_rfoot_tmp.block(2, 0, 4, NUM_QDOT);
	sejong::Matrix Jdot_rhand_task = Jdot_rhand_tmp.block(3, 0, 3, NUM_QDOT);

	sejong::Matrix Jdot(task_dim, NUM_QDOT); Jdot.setZero();
	Jdot.block(0, 0, 4, NUM_QDOT) = Jdot_rfoot_task;
	Jdot.block(4, 0, 3, NUM_QDOT) = Jdot_rhand_task;

	JtDotQdot = Jdot*qdot_state;
}

