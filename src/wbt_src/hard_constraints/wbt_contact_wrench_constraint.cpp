#include <wbt/hard_constraints/wbt_contact_wrench_constraint.hpp>
#include <Utils/utilities.hpp>
#include "valkyrie_definition.h"
#include <wbt/optimization_constants.hpp>

Contact_Wrench_Constraint::Contact_Wrench_Constraint(){
	Initialization();
}

Contact_Wrench_Constraint::Contact_Wrench_Constraint(Contact_List* contact_list_in, int index_in){
	Initialization();
}

Contact_Wrench_Constraint::Contact_Wrench_Constraint(Contact_List* contact_list_in, int index_in, double mu_in, double x_rec_width_in, double y_rec_width){
	Initialization();
}

Contact_Wrench_Constraint::~Contact_Wrench_Constraint(){
	std::cout << "Contact Wrench Constraint destructor called" << std::endl;
}

void Contact_Wrench_Constraint::Initialization(){
  std::cout << "Initialization called!" << std::endl; 
  initialize_Flow_Fupp();
}


void Contact_Wrench_Constraint::evaluate_constraint(const int &timestep, WBT_Opt_Variable_List& var_list, std::vector<double>& F_vec){

}

void Contact_Wrench_Constraint::evaluate_sparse_gradient(const int &timestep, WBT_Opt_Variable_List& var_list, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG){

}

void Contact_Wrench_Constraint::evaluate_sparse_A_matrix(const int &timestep, WBT_Opt_Variable_List& var_list, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA){

}

void Contact_Wrench_Constraint::initialize_Flow_Fupp(){
  for (size_t i = 0; i < num_constraints; i++){
    F_low.push_back(0.0);
    F_upp.push_back(OPT_INFINITY);    
  }
}
