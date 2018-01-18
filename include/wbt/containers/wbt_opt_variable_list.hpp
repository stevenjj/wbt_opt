#ifndef WBT_OPT_VARS_LIST_H
#define WBT_OPT_VARS_LIST_H

#include <Utils/wrap_eigen.hpp>
#include <vector>
#include <wbt/containers/wbt_opt_variable.hpp>

class WBT_Opt_Variable_List{
public:
	WBT_Opt_Variable_List();
	~WBT_Opt_Variable_List();	

	void append_variable(WBT_Opt_Variable* opt_variable);

	void get_var_states(const int timestep, sejong::Vector q_state, sejong::Vector qdot_state);	
	void get_var_reaction_forces(const int timestep, sejong::Vector Fr_state);		
	void get_var_keyframes(const int timestep, sejong::Vector keyframe_state);		

	int get_size();
	WBT_Opt_Variable* get_opt_variable(const int index);

	void populate_x(std::vector<double> x_out);

private:
	std::vector<WBT_Opt_Variable*> opt_var_list;
	//std::vector< int, std::vector<WBT_Opt_Variable*> > 
};

#endif