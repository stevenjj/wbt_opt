#include <wbt/hard_constraints/wbt_contact_wrench_constraint.hpp>
#include <Utils/utilities.hpp>
#include "valkyrie_definition.h"
#include <wbt/optimization_constants.hpp>

Contact_Wrench_Constraint::Contact_Wrench_Constraint(){
	Initialization();
}

Contact_Wrench_Constraint::Contact_Wrench_Constraint(Contact_List* contact_list_in, int index_in){
  setContact_List(contact_list_in);
  setContact_index(index_in);
  Initialization();
}

Contact_Wrench_Constraint::Contact_Wrench_Constraint(Contact_List* contact_list_in, int index_in, double mu_in, double x_rec_width_in, double y_rec_width_in){
  setContact_List(contact_list_in);
  setContact_index(index_in);  
  mu = mu_in;
  x_rec_width = x_rec_width_in;
  y_rec_width = y_rec_width_in;  
  Initialization();
}

Contact_Wrench_Constraint::~Contact_Wrench_Constraint(){
	std::cout << "Contact Wrench Constraint destructor called" << std::endl;
}

void Contact_Wrench_Constraint::Initialization(){
  std::cout << "Initialization called!" << std::endl; 
  initialize_Flow_Fupp();
}

void Contact_Wrench_Constraint::initialize_Flow_Fupp(){
  for (size_t i = 0; i < num_constraints; i++){
    F_low.push_back(0.0);
    F_upp.push_back(OPT_INFINITY);    
  }

  std::cout << "Contact Index:" << contact_index << std::endl;
  std::cout << "Contact Name:" <<  contact_list_obj->get_contact(contact_index)->contact_name << std::endl;  


}

void Contact_Wrench_Constraint::setContact_List(Contact_List* contact_list_in){
  contact_list_obj = contact_list_in;
}
void Contact_Wrench_Constraint::setContact_index(int index_in){
  contact_index = index_in;  
}  


void Contact_Wrench_Constraint::evaluate_constraint(const int &timestep, WBT_Opt_Variable_List& var_list, std::vector<double>& F_vec){
  Contact* current_contact = contact_list_obj->get_contact(contact_index);
}

void Contact_Wrench_Constraint::evaluate_sparse_gradient(const int &timestep, WBT_Opt_Variable_List& var_list, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG){

}

void Contact_Wrench_Constraint::evaluate_sparse_A_matrix(const int &timestep, WBT_Opt_Variable_List& var_list, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA){

}

void Contact_Wrench_Constraint::UpdateUf(const sejong::Vector &q_state, sejong::Matrix &Uf){


}

void Contact_Wrench_Constraint::setU(const double x, const double y, const double mu, sejong::Matrix & U){
  U = sejong::Matrix::Zero(17, 6);

  U(0, 5) = 1.;

  U(1, 3) = 1.; U(1, 5) = mu;
  U(2, 3) = -1.; U(2, 5) = mu;

  U(3, 4) = 1.; U(3, 5) = mu;
  U(4, 4) = -1.; U(4, 5) = mu;

  U(5, 0) = 1.; U(5, 5) = y;
  U(6, 0) = -1.; U(6, 5) = y;

  U(7, 1) = 1.; U(7, 5) = x;
  U(8, 1) = -1.; U(8, 5) = x;

  // Tau
  U(9, 0) = -mu; U(9, 1) = -mu; U(9, 2) = 1;
  U(9, 3) = y;   U(9, 4) = x;   U(9, 5) = (x + y)*mu;

  U(10, 0) = -mu; U(10, 1) = mu; U(10, 2) = 1;
  U(10, 3) = y;   U(10, 4) = -x; U(10, 5) = (x + y)*mu;

  U(11, 0) = mu; U(11, 1) = -mu; U(11, 2) = 1;
  U(11, 3) = -y; U(11, 4) = x;   U(11, 5) = (x + y)*mu;

  U(12, 0) = mu; U(12, 1) = mu; U(12, 2) = 1;
  U(12, 3) = -y; U(12, 4) = -x; U(12, 5) = (x + y)*mu;
  /////////////////////////////////////////////////
  U(13, 0) = -mu; U(13, 1) = -mu; U(13, 2) = -1;
  U(13, 3) = -y;  U(13, 4) = -x;  U(13, 5) = (x + y)*mu;

  U(14, 0) = -mu; U(14, 1) = mu; U(14, 2) = -1;
  U(14, 3) = -y;  U(14, 4) = x;  U(14, 5) = (x + y)*mu;

  U(15, 0) = mu; U(15, 1) = -mu; U(15, 2) = -1;
  U(15, 3) = y;  U(15, 4) = -x;  U(15, 5) = (x + y)*mu;

  U(16, 0) = mu; U(16, 1) = mu; U(16, 2) = -1;
  U(16, 3) = y;  U(16, 4) = x;  U(16, 5) = (x + y)*mu;
}