#ifndef WBT_WHOLEBODY_CONTROLLER_CONSTRAINT_H
#define WBT_WHOLEBODY_CONTROLLER_CONSTRAINT_H

#include <Utils/wrap_eigen.hpp>
#include <Utils/pseudo_inverse.hpp>

#include <string>
#include <iostream>

#include <wbt/hard_constraints/wbt_constraint_main.hpp>
#include <wbt/containers/wbt_wholebody_task_list.hpp>
#include <wbt/containers/wbt_contact_list.hpp>
#include "RobotModel.hpp"

class Wholebody_Controller_Constraint: public Constraint_Function{
public:
	Wholebody_Controller_Constraint();
	Wholebody_Controller_Constraint(WholeBody_Task_List* wb_task_list_input);	
	Wholebody_Controller_Constraint(WholeBody_Task_List* wb_task_list_input, Contact_List* contact_list_input);		
	~Wholebody_Controller_Constraint();
	RobotModel* robot_model;	

	WholeBody_Task_List* wb_task_list;
	Contact_List* contact_list;

	sejong::Matrix B_int;
	sejong::Vector c_int;

	sejong::Matrix Jc_int; // Contact Jacobian

	sejong::Matrix Sv; // Virtual Joints selection matrix
	sejong::Matrix Sa; // Actuated Joints selection matrix	


	int task_dim;
	int contact_dim;	

	float torque_limit;

	void set_task_list(WholeBody_Task_List* wb_task_list_input);
	void set_contact_list(Contact_List* contact_list_input);
	std::string constraint_name = "WBC constraint";


	void getB_c(const sejong::Vector &q, const sejong::Vector &qdot, sejong::Matrix &B_out, sejong::Vector &c_out);
	void get_Jc(const sejong::Vector &q, sejong::Matrix &Jc_out);

	void test_function();
	void test_function2(const sejong::Vector &q, const sejong::Vector &qdot, sejong::Matrix &B_out, sejong::Vector &c_out);

private:
	void Initialization();
	void initialize_Flow_Fupp();
	void _WeightedInverse(const sejong::Matrix & J, const sejong::Matrix & Winv, sejong::Matrix & Jinv){
	    sejong::Matrix lambda(J* Winv * J.transpose());
	    sejong::Matrix lambda_inv;
	    sejong::pseudoInverse(lambda, 0.0001, lambda_inv);
	    Jinv = Winv * J.transpose() * lambda_inv;
	  }
};
#endif