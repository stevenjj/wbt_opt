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

