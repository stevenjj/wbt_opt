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
	Constraint_Function();
	virtual ~Constraint_Function();
	virtual void evaluate_constraint(const int &timestep, WBT_Opt_Variable_List& var_list, std::vector<double>& F_vec);
	virtual void evaluate_sparse_gradient(const int &timestep, WBT_Opt_Variable_List& var_list, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG);
	virtual void evaluate_sparse_A_matrix(const int &timestep, WBT_Opt_Variable_List& var_list, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA);	

	// Each constraint containts its bounds
	std::vector<double> F_low;
	std::vector<double> F_upp;

	std::string constraint_name = "undefined constraint";	

	int constraint_index = -1; // Modified by the Constraint List Holder
	int constraint_size = 0; // Modified by the Object Constructor
	
	virtual int get_constraint_size();
	virtual int get_constraint_index();	

};


/*class Constraint_Function{
public:
	Constraint_Function(){}
	virtual ~Constraint_Function(){
		std::cout << "Constraint Function Destructor called" << std::endl;
	}
	virtual void evaluate_constraint(const int &timestep, WBT_Opt_Variable_List& var_list, std::vector<double>& F_vec) {}
	virtual void evaluate_sparse_gradient(const int &timestep, WBT_Opt_Variable_List& var_list, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG) {}
	virtual void evaluate_sparse_A_matrix(const int &timestep, WBT_Opt_Variable_List& var_list, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA) {}	

	// Each constraint containts its bounds
	std::vector<double> F_low;
	std::vector<double> F_upp;

	std::string constraint_name = "undefined constraint";	

	int constraint_index = -1; // Modified by the Constraint List Holder
	int constraint_size = 0; // Modified by the Object Constructor
	
	virtual int get_constraint_size(){ return constraint_size; }
	virtual int get_constraint_index(){ return constraint_index;}	

};*/



#endif