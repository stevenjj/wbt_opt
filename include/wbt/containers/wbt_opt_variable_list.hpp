#ifndef WBT_OPT_VARS_LIST_H
#define WBT_OPT_VARS_LIST_H

#include <Utils/wrap_eigen.hpp>
#include <vector>
#include <map>
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

	void update_x(std::vector<double> x_in);
	void populate_x(std::vector<double> x_out);

private:
	void add_variable_to_map(std::map<int, std::vector<WBT_Opt_Variable*> > &map_time_to_var_vec, WBT_Opt_Variable* opt_variable);

	std::vector<WBT_Opt_Variable*> opt_var_list;

	std::map<int, std::vector<WBT_Opt_Variable*> > timestep_to_q_state_vars;
	std::map<int, std::vector<WBT_Opt_Variable*> > timestep_to_qdot_state_vars;
	std::map<int, std::vector<WBT_Opt_Variable*> > timestep_to_xddot_vars;
	std::map<int, std::vector<WBT_Opt_Variable*> > timestep_to_Fr_vars;
	std::map<int, std::vector<WBT_Opt_Variable*> > timestep_to_keyframe_vars;
		
	//std::vector< int, std::vector<WBT_Opt_Variable*> > 
};

#endif