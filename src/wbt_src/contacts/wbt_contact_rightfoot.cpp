#include <wbt/contacts/wbt_contact_rightfoot.hpp>
#include "valkyrie_definition.h"

// Define RightFoot Contact ---------------------------------------------------------------
RightFoot_Contact::RightFoot_Contact(){
	robot_model_ = RobotModel::GetRobotModel();
	contact_dim = 6;
    contact_name = "Right Foot Contact";
}
RightFoot_Contact::~RightFoot_Contact(){}

void RightFoot_Contact::getContactJacobian(const sejong::Vector &q_state, sejong::Matrix & Jt){
	sejong::Matrix Jtmp;
    robot_model_->getFullJacobian(q_state, SJLinkID::LK_rightCOP_Frame, Jtmp);
    Jt = Jtmp; //Jtmp.block(3, 0, 3, NUM_QDOT);
}

void RightFoot_Contact::getContactJacobianDotQdot(const sejong::Vector &q_state, 
  							  			  const sejong::Vector &qdot_state, sejong::Vector & JtDotQdot){
	sejong::Matrix Jdot_tmp;    
    robot_model_->getFullJacobianDot(q_state, qdot_state, SJLinkID::LK_rightCOP_Frame, Jdot_tmp);
    sejong::Matrix Jdot_task = Jdot_tmp;//Jdot_tmp.block(3, 0, 3, NUM_QDOT);

	JtDotQdot = Jdot_task*qdot_state;    
}