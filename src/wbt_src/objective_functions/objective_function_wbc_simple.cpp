#include <wbt/objective_functions/objective_function_wbc_simple.hpp>
#include <Utils/utilities.hpp>

WBC_Objective_Function::WBC_Objective_Function(){}

void WBC_Objective_Function::set_var_list(WBT_Opt_Variable_List& var_list){
	num_q = var_list.get_num_q_vars();
	num_qdot = var_list.get_num_qdot_vars();
	num_Fr = var_list.get_num_xddot_vars();
	num_xddot = var_list.get_num_Fr_vars();
	num_kf =  var_list.get_num_keyframe_vars();
	num_timesteps =  var_list.total_timesteps;

	Q_mat = sejong::Matrix::Identity(num_Fr, num_Fr);
	N_mat = sejong::Matrix::Identity(num_xddot, num_xddot);
	R_mat = sejong::Matrix::Identity(num_kf, num_kf);

	Qdot_mat = sejong::Matrix::Identity(num_Fr, num_Fr);
	S_mat = sejong::Matrix::Identity(num_qdot, num_qdot);
}

void WBC_Objective_Function::evaluate_objective_function(WBT_Opt_Variable_List& var_list, double& result){
	sejong::Vector q_states;
	sejong::Vector qdot_states;
	sejong::Vector xddot_states;
	sejong::Vector Fr_states;	
	sejong::Vector kf_states;		

	sejong::Vector Fr_next_states;
	double dt = OPT_TIMESTEP;
	sejong::Vector Fr_dot;

	double cost = 0.0;
	// Evaluate state costs only
	for(size_t timestep = 0; timestep < num_timesteps; timestep++){
		var_list.get_var_reaction_forces(timestep, Fr_states);
		var_list.get_task_accelerations(timestep, xddot_states);
		var_list.get_var_states(timestep, q_states, qdot_states);
		var_list.get_var_keyframes(timestep, kf_states);		

		if (Fr_states.size() > 0){
			cost += Fr_states.transpose()*Q_mat*Fr_states;
		}
		if (xddot_states.size() > 0){
			cost += xddot_states.transpose()*N_mat*xddot_states;
		}
		// if (qdot_states.size() > 0){
		// 	//std::cout << "qdot_states.size() = " << qdot_states.size() << std::endl;
		// 	cost += qdot_states.transpose()*S_mat*qdot_states;			
		// }
		if (kf_states.size() > 0){
			// get desired keyframes.
		}

	}
	
	// Evaluate time derivative costs
	for(size_t timestep = 1; timestep < num_timesteps; timestep++){
		var_list.get_var_reaction_forces(timestep, Fr_next_states);		
		Fr_dot = (Fr_next_states - Fr_states)/dt ;
		cost += Fr_dot.transpose()*Qdot_mat*Fr_dot;
	}	
	result = cost;


}

void WBC_Objective_Function::evaluate_objective_gradient(WBT_Opt_Variable_List& var_list, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG){
  int m = var_list.get_size_timedependent_vars(); // var_list.get_num_time_dependent_vars
  int T = var_list.total_timesteps; // var_list.get_total_timesteps() Total timestep
  int k = var_list.get_num_keyframe_vars();
  
  int i_local = 0;               // specify i starting index
  int local_j_offset = 0;
  int j_local = 0;


  sejong::Vector xddot_states;
  sejong::Vector Fr_states;  
  sejong::Matrix F_dxddot = (N_mat + N_mat.transpose());
  sejong::Matrix F_dFr = (Q_mat + Q_mat.transpose());

  for (size_t timestep = 0; timestep < T; timestep++){
  	  // Get Gradient w.r.t task accelerations
  	  local_j_offset = m*timestep + var_list.get_num_q_vars() + var_list.get_num_qdot_vars();
	  j_local = local_j_offset;// j = (total_j_size*timestep) + var_states_size + task_accelerations size // specify j starting index
	  var_list.get_task_accelerations(timestep, xddot_states);

	  F_dxddot = xddot_states.transpose()*(N_mat + N_mat.transpose());

	  // sejong::pretty_print(xddot_states, std::cout, "xddot_states");
	  // sejong::pretty_print(F_dxddot, std::cout, "F_dxddot");

	  for(size_t j = 0; j < F_dxddot.cols(); j++){
	  	iG.push_back(i_local);
	  	G.push_back(F_dxddot(0, j));  	
	  	jG.push_back(j + j_local);
	  	//std::cout << "J + j_local = " << j + j_local << std::endl;
	  }


	  // Get Gradient w.r.t Reaction Force
  	  local_j_offset = m*timestep + var_list.get_num_q_vars() + var_list.get_num_qdot_vars() + var_list.get_num_xddot_vars();
	  var_list.get_var_reaction_forces(timestep, Fr_states);

	  F_dFr = Fr_states.transpose()*(Q_mat + Q_mat.transpose());
	  for(size_t j = 0; j < F_dFr.cols(); j++){
	  	iG.push_back(i_local);
	  	G.push_back(F_dFr(0, j));  	
	  	jG.push_back(j + j_local);
	  } 

  }
  // Go through F_dFr and push back values to G, iGfun, jGfun 

}

void WBC_Objective_Function::evaluate_sparse_A_matrix(WBT_Opt_Variable_List& var_list, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA){

}	


void WBC_Objective_Function::setQ_vals(const int &i, const int &j, const double &value){
	if ( (i < Q_mat.rows()) && j < Q_mat.cols() ) {
		Q_mat(i,j) = value;		
	}
}