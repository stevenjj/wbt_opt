#include <wbt/hard_constraints/wbt_contact_lcp_constraint.hpp>
#include <Utils/utilities.hpp>
#include "valkyrie_definition.h"
#include <wbt/optimization_constants.hpp>

Contact_LCP_Constraint::Contact_LCP_Constraint(){}
Contact_LCP_Constraint::~Contact_LCP_Constraint(){}

void Contact_LCP_Constraint::Initialization(){
	initialize_Flow_Fupp();	
}

void Contact_LCP_Constraint::initialize_Flow_Fupp(){
	// Phi(q)*Fr = 0
	// Phi(q) >= 0
	F_low.push_back(0.0);	
	F_low.push_back(0.0);

	F_upp.push_back(0.0);
	F_upp.push_back(OPT_INFINITY);			
}


void Contact_LCP_Constraint::evaluate_constraint(const int &timestep, WBT_Opt_Variable_List& var_list, std::vector<double>& F_vec){}
void Contact_LCP_Constraint::evaluate_sparse_gradient(const int &timestep, WBT_Opt_Variable_List& var_list, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG){}
void Contact_LCP_Constraint::evaluate_sparse_A_matrix(const int &timestep, WBT_Opt_Variable_List& var_list, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA){}
