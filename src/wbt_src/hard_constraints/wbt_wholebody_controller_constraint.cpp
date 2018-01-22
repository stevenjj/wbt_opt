#include <wbt/hard_constraints/wbt_wholebody_controller_constraint.hpp>
#include <Utils/utilities.hpp>
#include "valkyrie_definition.h"

Wholebody_Controller_Constraint::Wholebody_Controller_Constraint(){
	Initialization();
}

Wholebody_Controller_Constraint::Wholebody_Controller_Constraint(WholeBody_Task_List* wb_task_list_input){
	Initialization();
	set_task_list(wb_task_list_input);
}

Wholebody_Controller_Constraint::Wholebody_Controller_Constraint(WholeBody_Task_List* wb_task_list_input, Contact_List* contact_list_input){
	Initialization();
	set_task_list(wb_task_list_input);
	set_contact_list(contact_list_input);
}

Wholebody_Controller_Constraint::~Wholebody_Controller_Constraint(){
	std::cout << "WCC destructor called" << std::endl;
}


void Wholebody_Controller_Constraint::Initialization(){
  robot_model = RobotModel::GetRobotModel();	
  Sv.resize(NUM_VIRTUAL, NUM_QDOT);
  Sa.resize(NUM_ACT_JOINT, NUM_QDOT);
  Sv.setZero();
  Sa.setZero();

  Sv.block(0,0, NUM_VIRTUAL, NUM_VIRTUAL) = sejong::Matrix::Identity(NUM_VIRTUAL, NUM_VIRTUAL);
  Sa.block(0, NUM_VIRTUAL, NUM_ACT_JOINT, NUM_ACT_JOINT) = sejong::Matrix::Identity(NUM_ACT_JOINT, NUM_ACT_JOINT);

}

void Wholebody_Controller_Constraint::set_task_list(WholeBody_Task_List* wb_task_list_input){
	std::cout << "[WBC Constraint] Processing Task List" << std::endl;

	wb_task_list = wb_task_list_input;	
	task_dim = 0;
	for(size_t i = 0; i < wb_task_list->get_size(); i++){
		task_dim += wb_task_list->get_task(i)->task_dim;
	}

	std::cout << "[WBC Constraint] Task List Processed" << std::endl;
	std::cout << "[WBC Constraint] Task List size: " << wb_task_list->get_size() << std::endl;
	std::cout << "[WBC Constraint] Task Dimension size: " << task_dim << std::endl;
}



void Wholebody_Controller_Constraint::set_contact_list(Contact_List* contact_list_input){
	std::cout << "[WBC Constraint] Processing Contact List" << std::endl;

	contact_list = contact_list_input;

  contact_dim = 0;
  for(size_t i = 0; i < contact_list->get_size(); i++){
    contact_dim += contact_list->get_contact(i)->contact_dim;
  }  

	std::cout << "[WBC Constraint] Contact List Processed" << std::endl;
  std::cout << "[WBC Constraint] Contact List size: " << contact_list->get_size() << std::endl;
	std::cout << "[WBC Constraint] Contact Dimension: " << contact_dim << std::endl;
}

void Wholebody_Controller_Constraint::test_function(){
	std::cout << "[WBC Constraint] Test function called" << std::endl;
}

void Wholebody_Controller_Constraint::test_function2(const sejong::Vector &q, const sejong::Vector &qdot, sejong::Matrix &B_out, sejong::Vector &c_out){
	getB_c(q, qdot, B_out, c_out);	
	get_Jc(q, Jc_int);
	sejong::pretty_print(Jc_int, std::cout, "WBDC: Jc_int");
}

void Wholebody_Controller_Constraint::getB_c(const sejong::Vector &q, const sejong::Vector &qdot, sejong::Matrix &B_out, sejong::Vector &c_out){
  sejong::Matrix B;
  sejong::Vector c;  

  sejong::Matrix Ainv;
  sejong::Matrix Jt, JtPre;
  sejong::Matrix Jt_inv, JtPre_inv;
  sejong::Vector JtDotQdot;
  sejong::Vector xddot;
  sejong::Matrix Npre;
  sejong::Matrix I_JtPreInv_Jt;
  Task* task = wb_task_list->get_task(0);

  int tot_task_size(0);

  robot_model->UpdateModel(q, qdot); // Update Model. This call should be moved so that it's only called once.
  robot_model->getInverseMassInertia(Ainv);
  task->getTaskJacobian(q, Jt);
  task->getTaskJacobianDotQdot(q, qdot, JtDotQdot);

  _WeightedInverse(Jt, Ainv, Jt_inv);
  B = Jt_inv;
  c = Jt_inv * JtDotQdot;


  Npre = sejong::Matrix::Identity(NUM_QDOT, NUM_QDOT) - Jt_inv * Jt;
  tot_task_size += task->task_dim;



  for(int i(1); i<wb_task_list->get_size(); ++i){
    // Obtaining Task
    task = wb_task_list->get_task(i);

    task->getTaskJacobian(q, Jt);
    task->getTaskJacobianDotQdot(q, qdot, JtDotQdot);   
    JtPre = Jt * Npre;
    _WeightedInverse(JtPre, Ainv, JtPre_inv);
    I_JtPreInv_Jt = sejong::Matrix::Identity(NUM_QDOT, NUM_QDOT) - JtPre_inv * Jt;

    // B matrix building
    B.conservativeResize(NUM_QDOT, tot_task_size + task->task_dim);
    B.block(0, 0, NUM_QDOT, tot_task_size) =
      I_JtPreInv_Jt * B.block(0, 0, NUM_QDOT, tot_task_size);
    B.block(0, tot_task_size, NUM_QDOT, task->task_dim) = JtPre_inv;
    // c vector building
    c = I_JtPreInv_Jt * c - JtPre_inv * JtDotQdot;

    // Build for Next
    Npre = Npre * ( sejong::Matrix::Identity(NUM_QDOT, NUM_QDOT) - JtPre_inv * JtPre);
    tot_task_size += task->task_dim;

  }

/*  sejong::pretty_print(B, std::cout, "WBDC: B");
  sejong::pretty_print(c, std::cout, "WBDC: c");*/

  B_out = B;
  c_out = c;
}

// Gets the contact Jacobian Jc
void Wholebody_Controller_Constraint::get_Jc(const sejong::Vector &q, sejong::Matrix &Jc_out){
  Contact* contact;
  sejong::Matrix Jc;
  sejong::Matrix Jtmp;
  int total_contact_dim = 0;
  for(int i = 0; i < contact_list->get_size(); i++){ 	
  	contact = contact_list->get_contact(i);
  	contact->getContactJacobian(q, Jtmp); 

  	Jc.conservativeResize(total_contact_dim + contact->contact_dim, NUM_QDOT);
  	Jc.block(total_contact_dim, 0, contact->contact_dim, NUM_QDOT) = Jtmp;

  	total_contact_dim += contact->contact_dim;
  }

  Jc_out = Jc;
}

/*

get states q, qdot
get xddot task accelerations
get Fr reaction Force

qddot = Bxddot + c
Aqddot + b + g - Jc^T *Fr */

