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
	void compute_size_time_dep_vars(); // This function needs to have been called at least once if a getter function is used.


	WBT_Opt_Variable* get_opt_variable(const int index);

	void get_var_states(const int &timestep, sejong::Vector &q_state, sejong::Vector &qdot_state);	
	void get_task_accelerations(const int &timestep, sejong::Vector &xddot);		
	void get_var_reaction_forces(const int &timestep, sejong::Vector &Fr_state);		
	void get_var_keyframes(const int &timestep, sejong::Vector &keyframe_state);		
	void get_var_knotpoint_dt(const int &timestep, double &h_dt);


	int get_size();
	int get_size_timedependent_vars();

	void update_x(std::vector<double> &x_in);	 // To do
	void populate_x(std::vector<double> &x_out); // To do

	int total_timesteps;

	int get_num_q_vars();
	int get_num_qdot_vars();
	int get_num_xddot_vars();
	int get_num_Fr_vars();
	int get_num_keyframe_vars();		

	int get_num_var_knotpoint_dt();
private:
	void add_variable_to_map(std::map<int, std::vector<WBT_Opt_Variable*> > &map_time_to_var_vec, WBT_Opt_Variable* opt_variable);

	void convert_to_vector(const int &timestep, 
						   std::map<int, std::vector<WBT_Opt_Variable*> > &map_time_to_var_vec,
						   sejong::Vector &vec_out);

	std::vector<WBT_Opt_Variable*> opt_var_list;

	std::map<int, std::vector<WBT_Opt_Variable*> > timestep_to_q_state_vars;
	std::map<int, std::vector<WBT_Opt_Variable*> > timestep_to_qdot_state_vars;
	std::map<int, std::vector<WBT_Opt_Variable*> > timestep_to_xddot_vars;
	std::map<int, std::vector<WBT_Opt_Variable*> > timestep_to_Fr_vars;
	std::map<int, std::vector<WBT_Opt_Variable*> > timestep_to_keyframe_vars;

	std::vector<WBT_Opt_Variable*> timestep_to_knotpoint_dt;
	
	int count_num_vars_in_map(const int &timestep, std::map<int, std::vector<WBT_Opt_Variable*> > &map_time_to_var_vec);
	//std::vector< int, std::vector<WBT_Opt_Variable*> > 
	int num_timedep_vars = 0; 

	int num_q_vars = 0;
	int num_qdot_vars = 0;
	int num_xddot_vars = 0;
	int num_Fr_vars = 0;
	int num_keyframe_vars = 0;

	int num_knotpoint_dt_vars = 0;	

};

#endif