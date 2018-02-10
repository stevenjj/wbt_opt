#ifndef WBT_CONTACT_LCP_CONSTRAINT_H
#define WBT_CONTACT_LCP_CONSTRAINT_H

#include <Utils/wrap_eigen.hpp>

#include <string>
#include <iostream>

#include <wbt/hard_constraints/wbt_constraint_main.hpp>
#include <wbt/containers/wbt_contact_list.hpp>

#include "RobotModel.hpp"

class Contact_LCP_Constraint: public Constraint_Function{
public:
	Contact_LCP_Constraint();
	~Contact_LCP_Constraint();

	RobotModel* robot_model;	

	void evaluate_constraint(const int &timestep, WBT_Opt_Variable_List& var_list, std::vector<double>& F_vec);
	void evaluate_sparse_gradient(const int &timestep, WBT_Opt_Variable_List& var_list, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG);
	void evaluate_sparse_A_matrix(const int &timestep, WBT_Opt_Variable_List& var_list, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA);

private:
	const int num_constraints = 1;
	Contact_List* contact_list_obj;
	int contact_index = -1;	

	void Initialization();
	void initialize_Flow_Fupp();

	void UpdateModel(const sejong::Vector &q, const sejong::Vector &qdot,
                      sejong::Matrix &A_out, sejong::Vector &grav_out, sejong::Vector &cori_out);
	void UpdateModel(const int timestep, const sejong::Vector &q, const sejong::Vector &qdot,
                      sejong::Matrix &A_out, sejong::Vector &grav_out, sejong::Vector &cori_out);	
};
#endif