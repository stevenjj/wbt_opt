#ifndef WBT_CONTACT_WRENCH_CONSTRAINT_H
#define WBT_CONTACT_WRENCH_CONSTRAINT_H

#include <Utils/wrap_eigen.hpp>

#include <string>
#include <iostream>


#include <wbt/hard_constraints/wbt_constraint_main.hpp>
#include <wbt/containers/wbt_contact_list.hpp>

#include "RobotModel.hpp"

class Contact_Wrench_Constraint: public Constraint_Function{
public:
	Contact_Wrench_Constraint();
	Contact_Wrench_Constraint(Contact_List* contact_list_in, int index_in);	
	Contact_Wrench_Constraint(Contact_List* contact_list_in, int index_in, double mu_in, double x_rec_width_in, double y_rec_width_in);		
	~Contact_Wrench_Constraint();

	RobotModel* robot_model;	

	
	void setContact_List(Contact_List* contact_list_in);
	void setContact_index(int index_in);	

	void evaluate_constraint(const int &timestep, WBT_Opt_Variable_List& var_list, std::vector<double>& F_vec);
	void evaluate_sparse_gradient(const int &timestep, WBT_Opt_Variable_List& var_list, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG);
	void evaluate_sparse_A_matrix(const int &timestep, WBT_Opt_Variable_List& var_list, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA);

private:
	const int num_constraints = 17;
	Contact_List* contact_list_obj;
	int contact_index = -1;
	double mu;
	double x_rec_width;
	double y_rec_width;
	sejong::Matrix U_int;


	void Initialization();
	void initialize_Flow_Fupp();



	void UpdateUf(const sejong::Vector &q_state, sejong::Matrix &Uf);
	void setU(const double x, const double y, const double mu, sejong::Matrix & U);

	void UpdateModel(const sejong::Vector &q, const sejong::Vector &qdot,
                      sejong::Matrix &A_out, sejong::Vector &grav_out, sejong::Vector &cori_out);
	void UpdateModel(const int timestep, const sejong::Vector &q, const sejong::Vector &qdot,
                      sejong::Matrix &A_out, sejong::Vector &grav_out, sejong::Vector &cori_out);	
};
#endif