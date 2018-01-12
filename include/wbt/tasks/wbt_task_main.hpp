#ifndef WBT_TASK_PARENT_H
#define WBT_TASK_PARENT_H
#include <Utils/wrap_eigen.hpp>
#include <string>

class Task{
public:
  Task(){}
  virtual ~Task(){}
  virtual void getTaskJacobian(const sejong::Vector &q_state, sejong::Matrix & Jt) = 0;
  virtual void getTaskJacobianDotQdot(const sejong::Vector &q_state, 
  							  		  const sejong::Vector &qdot_state, sejong::Vector & JtDotQdot) = 0;
  std::string task_name = "Undefined Task";
  int task_dim;
};

#endif