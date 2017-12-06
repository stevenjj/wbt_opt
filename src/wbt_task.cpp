#include "wbt_task.h"
#include "valkyrie_definition.h"

// Define COM Task ---------------------------------------------------------------
COM_Task::COM_Task(){
  robot_model_ = RobotModel::GetRobotModel();	
  task_dim = 3;
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

// Define LeftFoot Task ---------------------------------------------------------------
LeftFootTask::LeftFootTask(){
	robot_model_ = RobotModel::GetRobotModel();
	task_dim = 3;
}
LeftFootTask::~LeftFootTask(){}

void LeftFootTask::getTaskJacobian(const sejong::Vector &q_state, sejong::Matrix & Jt){
	sejong::Matrix Jtmp;
    robot_model_->getFullJacobian(q_state, SJLinkID::LK_leftCOP_Frame, Jtmp);
    Jt = Jtmp.block(3, 0, 3, NUM_QDOT);
}

void LeftFootTask::getTaskJacobianDotQdot(const sejong::Vector &q_state, 
  							  			  const sejong::Vector &qdot_state, sejong::Vector & JtDotQdot){
	sejong::Matrix Jdot_tmp;    
    robot_model_->getFullJacobianDot(q_state, qdot_state, SJLinkID::LK_leftCOP_Frame, Jdot_tmp);
    sejong::Matrix Jdot_task = Jdot_tmp.block(3, 0, 3, NUM_QDOT);

	JtDotQdot = Jdot_task*qdot_state;    
}

// Define RightFoot Task ---------------------------------------------------------------
RightFoot_Hand_Task::RightFoot_Hand_Task(){
	robot_model_ = RobotModel::GetRobotModel();
	task_dim = 7; // 4 for the right foot, and 3 for the right hand 	
}
RightFoot_Hand_Task::~RightFoot_Hand_Task(){}

void RightFoot_Hand_Task::getTaskJacobian(const sejong::Vector &q_state, sejong::Matrix & Jt){
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

void RightFoot_Hand_Task::getTaskJacobianDotQdot(const sejong::Vector &q_state, 
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

