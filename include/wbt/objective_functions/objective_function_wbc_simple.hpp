#ifndef WBT_OBJ_FUNC_WBC_SIMPLE_H
#define WBT_OBJ_FUNC_WBC_SIMPLE_H

#include <wbt/objective_functions/objective_function_main.hpp>
class WBC_Objective_Function: public Objective_Function{
public:
	WBC_Objective_Function();
	virtual ~WBC_Objective_Function(){
		std::cout << "WBC Objective Function Destructor called" << std::endl;
	}


	void evaluate_objective_function(WBT_Opt_Variable_List& var_list, double& result);
	void evaluate_objective_gradient(WBT_Opt_Variable_List& var_list, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG);
	void evaluate_sparse_A_matrix(WBT_Opt_Variable_List& var_list, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA);	
	void set_var_list(WBT_Opt_Variable_List& var_list);

	void set_keyframes(); // soft keyframe list here.

	void setQ_vals(const int& i, const int& j, double& value);

	std::string objective_function_name = "wbc_simple_objective function";	
	sejong::Matrix Q_mat; // Cost Matrix for reaction forces
	sejong::Matrix Qdot_mat; // Cost Matrix for time derivative reaction forces

	sejong::Matrix N_mat; // Cost Matrix for task acceleration
	sejong::Matrix R_mat; // Cost Matrix for keyframes

	sejong::Matrix S_mat; // Cost Matrix for qdot states

	int num_q;
	int num_qdot;
	int num_xddot;
	int num_Fr;
	int num_kf;
	int num_timesteps;

};



#endif