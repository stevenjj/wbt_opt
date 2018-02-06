#ifndef WBT_CONTACT_PARENT_H
#define WBT_CONTACT_PARENT_H
#include <Utils/wrap_eigen.hpp>
#include <string>

class Contact{
public:
  Contact(){}
  virtual ~Contact(){}
  virtual void getContactJacobian(const sejong::Vector &q_state, sejong::Matrix & Jt) = 0;
  virtual void getContactJacobianDotQdot(const sejong::Vector &q_state, 
  							  		  const sejong::Vector &qdot_state, sejong::Vector & JtDotQdot) = 0;
  std::string contact_name = "Undefined Contact";
  int contact_dim;
  int contact_link_id;
};

#endif