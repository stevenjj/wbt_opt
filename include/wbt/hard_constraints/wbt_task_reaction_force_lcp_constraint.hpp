#ifndef WBT_TASK_REACTION_FORCE_LCP_CONSTRAINT_H
#define WBT_TASK_REACTION_FORCE_LCP_CONSTRAINT_H

#include <Utils/wrap_eigen.hpp>

#include <string>
#include <iostream>

#include <wbt/hard_constraints/wbt_constraint_main.hpp>
#include <wbt/containers/wbt_contact_list.hpp>
#include <wbt/containers/wbt_wholebody_task_list.hpp>

#include "RobotModel.hpp"

class Task_Reaction_Force_LCP_Constraint: public Constraint_Function{
public:
	Task_Reaction_Force_LCP_Constraint();
	Task_Reaction_Force_LCP_Constraint(WholeBody_Task_List* wb_task_list_in, Contact_List* contact_list_in, int index_in);		
	~Task_Reaction_Force_LCP_Constraint();

	RobotModel* robot_model;	

	void setContact_List(Contact_List* contact_list_in);
	void setTask_List(WholeBody_Task_List* wb_task_list_in);	

	void setTask_index(int index_in);	
	void setContact_index(int index_in);	

	void evaluate_constraint(const int &timestep, WBT_Opt_Variable_List& var_list, std::vector<double>& F_vec);
	void evaluate_sparse_gradient(const int &timestep, WBT_Opt_Variable_List& var_list, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG);
	void evaluate_sparse_A_matrix(const int &timestep, WBT_Opt_Variable_List& var_list, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA);

private:
	const int num_constraints = 1;
	Contact_List* contact_list_obj;
	WholeBody_Task_List* wb_task_list_obj;

	int task_index = -1;	
	int contact_index = -1;	

	void Initialization();
	void initialize_Flow_Fupp();

	void UpdateModel(const sejong::Vector &q, const sejong::Vector &qdot,
                      sejong::Matrix &A_out, sejong::Vector &grav_out, sejong::Vector &cori_out);
	void UpdateModel(const int timestep, const sejong::Vector &q, const sejong::Vector &qdot,
                      sejong::Matrix &A_out, sejong::Vector &grav_out, sejong::Vector &cori_out);	
};
#endif