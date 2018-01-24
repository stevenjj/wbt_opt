#include <wbt/containers/wbt_opt_variable_list.hpp>

#include <iostream>
WBT_Opt_Variable_List::WBT_Opt_Variable_List():total_timesteps(0){}
WBT_Opt_Variable_List::~WBT_Opt_Variable_List(){
	for(size_t i = 0; i < opt_var_list.size(); i++){
		delete opt_var_list[i];
	}
	opt_var_list.clear();
	std::cout << "Optimization Variable List Destructor Called" << std::endl;
}

void WBT_Opt_Variable_List::append_variable(WBT_Opt_Variable* opt_variable){
	opt_var_list.push_back(opt_variable);

	if (opt_variable->type == VAR_TYPE_Q){
		add_variable_to_map(timestep_to_q_state_vars, opt_variable);
	}else if(opt_variable->type == VAR_TYPE_QDOT){
		add_variable_to_map(timestep_to_qdot_state_vars, opt_variable);
	}else if(opt_variable->type == VAR_TYPE_TA){
		add_variable_to_map(timestep_to_xddot_vars, opt_variable);		
	}else if(opt_variable->type == VAR_TYPE_FR){
		add_variable_to_map(timestep_to_Fr_vars, opt_variable);		
	}else if(opt_variable->type == VAR_TYPE_KF){
		add_variable_to_map(timestep_to_keyframe_vars, opt_variable);		
	}

/*	std::cout << "Size of Q map[timestep = 0]:    " << timestep_to_q_state_vars[0].size() << std::endl;
	std::cout << "Size of Qdot map[timestep = 0]: " << timestep_to_qdot_state_vars[0].size() << std::endl;	
	std::cout << "Size of xddot map[timestep = 0]:    " << timestep_to_xddot_vars[0].size() << std::endl;
	std::cout << "Size of Fr map[timestep = 0]: " << timestep_to_Fr_vars[0].size() << std::endl;		
	std::cout << "Size of KF map[timestep = 0]: " << timestep_to_keyframe_vars[0].size() << std::endl;	*/		
}


int WBT_Opt_Variable_List::get_size(){
	return opt_var_list.size();
}

WBT_Opt_Variable* WBT_Opt_Variable_List::get_opt_variable(const int index){
	if ((index >= 0) && (index < opt_var_list.size())){
		return opt_var_list[index];
	}else{
		std::cerr << "Error retrieving optimization variable. Index is out of bounds" << std::endl;
		throw "invalid_index";
	}
}


void WBT_Opt_Variable_List::add_variable_to_map(std::map<int, std::vector<WBT_Opt_Variable*>  > &map_time_to_var_vec, WBT_Opt_Variable* opt_variable){
	int timestep = opt_variable->time_step;
	if (map_time_to_var_vec.count(timestep) != 0){
		// If key is found add this optimization variable to the current time step
		map_time_to_var_vec[timestep].push_back(opt_variable);
	}else{
		// If key is not found, construct a new vector and add it to the map.
		std::vector<WBT_Opt_Variable*> var_vec = {opt_variable};
		map_time_to_var_vec.insert(  std::make_pair(timestep, var_vec) );
	}
}

void WBT_Opt_Variable_List::get_var_states(const int &timestep, sejong::Vector &q_state, sejong::Vector &qdot_state){
		convert_to_vector(timestep, timestep_to_q_state_vars, q_state);
		convert_to_vector(timestep, timestep_to_qdot_state_vars, qdot_state);		

/*		for(size_t i = 0; i < q_state.size(); i++){
			std::cout << " q_vec[i] val = " << q_state[i] << std::endl;
		}*/
/*		for(size_t i = 0; i < qdot_state.size(); i++){
			std::cout << " qdot_vec[i] val = " << qdot_state[i] << std::endl;
		}		*/

}

void WBT_Opt_Variable_List::get_task_accelerations(const int &timestep, sejong::Vector &xddot){
		convert_to_vector(timestep, timestep_to_xddot_vars, xddot);
/*		for(size_t i = 0; i < xddot.size(); i++){
			std::cout << " xddot[i] val = " << xddot[i] << std::endl;
		}		*/

}

void WBT_Opt_Variable_List::get_var_reaction_forces(const int &timestep, sejong::Vector &Fr_state){
		convert_to_vector(timestep, timestep_to_Fr_vars, Fr_state);
/*		for(size_t i = 0; i < Fr_state.size(); i++){
			std::cout << " Fr_state[i] val = " << Fr_state[i] << std::endl;
		}		*/
		
}

void WBT_Opt_Variable_List::convert_to_vector(const int &timestep, 
						  	 	   			  std::map<int, std::vector<WBT_Opt_Variable*> > &map_time_to_var_vec,
						   				 	  sejong::Vector &vec_out){
	
	std::map<int, std::vector<WBT_Opt_Variable*> >::iterator it;
	it = map_time_to_var_vec.find(timestep);

	// if the key in the map has been found
	if (it != map_time_to_var_vec.end()){
		int size_of_vec = it->second.size(); //is the size of std::vector<WBT_Opt_Variable*>
		vec_out.resize(size_of_vec);

		// Populate vector
		for(size_t i = 0; i < it->second.size(); i++){
			vec_out[i] = it->second[i]->value;
		}

	}


}