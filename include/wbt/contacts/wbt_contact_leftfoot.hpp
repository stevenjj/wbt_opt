#ifndef WBT_CONTACT_LEFTFOOT_H
#define WBT_CONTACT_LEFTFOOT_H

#include <wbt/contacts/wbt_contact_main.hpp>
#include "RobotModel.hpp"

class LeftFoot_Contact: public Contact{
public:
	LeftFoot_Contact();
	~LeftFoot_Contact();	
	RobotModel* robot_model_;

	void getContactJacobian(const sejong::Vector &q_state, sejong::Matrix & Jt);
    void getContactJacobianDotQdot(const sejong::Vector &q_state, 
  							  const sejong::Vector &qdot_state, sejong::Vector & JtDotQdot);
};

#endif