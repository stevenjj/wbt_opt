#include <wbt/hard_constraints/wbt_task_reaction_force_lcp_constraint.hpp>
#include <Utils/utilities.hpp>
#include "valkyrie_definition.h"
#include <wbt/optimization_constants.hpp>
#include <cmath>

Task_Reaction_Force_LCP_Constraint::Task_Reaction_Force_LCP_Constraint(){
}

Task_Reaction_Force_LCP_Constraint::~Task_Reaction_Force_LCP_Constraint(){}

Task_Reaction_Force_LCP_Constraint::Task_Reaction_Force_LCP_Constraint(WholeBody_Task_List* wb_task_list_in, Contact_List* contact_list_in, int index_in){
}	


void Task_Reaction_Force_LCP_Constraint::Initialization(){

}

void Task_Reaction_Force_LCP_Constraint::initialize_Flow_Fupp(){
	
}

void Task_Reaction_Force_LCP_Constraint::evaluate_constraint(const int &timestep, WBT_Opt_Variable_List& var_list, std::vector<double>& F_vec){}
void Task_Reaction_Force_LCP_Constraint::evaluate_sparse_gradient(const int &timestep, WBT_Opt_Variable_List& var_list, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG){}
void Task_Reaction_Force_LCP_Constraint::evaluate_sparse_A_matrix(const int &timestep, WBT_Opt_Variable_List& var_list, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA){}

