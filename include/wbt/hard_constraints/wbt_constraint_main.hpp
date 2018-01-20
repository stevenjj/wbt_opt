#ifndef WBT_CONSTRAINT_FUNC_H
#define WBT_CONSTRAINT_FUNC_H

#include <Utils/wrap_eigen.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <wbt/containers/wbt_opt_variable.hpp>
#include <wbt/containers/wbt_opt_variable_list.hpp>

class Constraint_Function{
public:
	Constraint_Function(){}
	virtual ~Constraint_Function(){
		std::cout << "Constraint Function Destructor called" << std::endl;
	}
	virtual void evaluate_constraint(const WBT_Opt_Variable_List& var_list, sejong::Vector& F_vec) {}
	virtual void evaluate_sparse_gradient(const WBT_Opt_Variable_List& var_list, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG) {}

	// Each constraint containts its bounds
	std::vector<double> F_low;
	std::vector<double> F_upp;

	std::string constraint_name = "undefined constraint";	

};

/*
class Constraint_List{
	Constraint_List();
	~Constraint_List();
	int timesteps;

	int total_constraint_size; // = 1 + num_constraints*timesteps + hard_keyframe constraints
	int total_constraint_size; // = 1 + num_constraints*timesteps + hard_keyframe constraints

	std::vector<Constraint_Function*> constraint_list;
	//std::vector<int, std::vector<Constraint_Function*> > timestep_constraint_list

	void evaluate_all_constraints(std::vector<double> F);
	// preset size of F
	//	for (size_t t = 0; t < timesteps; t++){
	//		F[t*constraint_size]
	//	}
	
};*/

#endif