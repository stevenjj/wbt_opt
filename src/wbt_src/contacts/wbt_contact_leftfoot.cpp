#include <wbt/contacts/wbt_contact_leftfoot.hpp>
#include "valkyrie_definition.h"

// Define LeftFoot Contact ---------------------------------------------------------------
LeftFoot_Contact::LeftFoot_Contact(){
	robot_model_ = RobotModel::GetRobotModel();
	contact_dim = 6;
    contact_name = "Left Foot Contact";
    contact_link_id = SJLinkID::LK_leftCOP_Frame;
}
LeftFoot_Contact::~LeftFoot_Contact(){}

void LeftFoot_Contact::getContactJacobian(const sejong::Vector &q_state, sejong::Matrix & Jt){
	sejong::Matrix Jtmp;
    robot_model_->getFullJacobian(q_state, SJLinkID::LK_leftCOP_Frame, Jtmp);
    Jt = Jtmp; //Jtmp.block(3, 0, 3, NUM_QDOT);
}

void LeftFoot_Contact::getContactJacobianDotQdot(const sejong::Vector &q_state, 
  							  			  const sejong::Vector &qdot_state, sejong::Vector & JtDotQdot){
	sejong::Matrix Jdot_tmp;    
    robot_model_->getFullJacobianDot(q_state, qdot_state, SJLinkID::LK_leftCOP_Frame, Jdot_tmp);
    sejong::Matrix Jdot_task = Jdot_tmp;//Jdot_tmp.block(3, 0, 3, NUM_QDOT);

	JtDotQdot = Jdot_task*qdot_state;    
}
