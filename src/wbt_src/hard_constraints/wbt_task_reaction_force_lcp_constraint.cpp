#include <wbt/hard_constraints/wbt_task_reaction_force_lcp_constraint.hpp>
#include <Utils/utilities.hpp>
#include "valkyrie_definition.h"
#include <wbt/optimization_constants.hpp>
#include <cmath>

Task_Reaction_Force_LCP_Constraint::Task_Reaction_Force_LCP_Constraint(){
}

Task_Reaction_Force_LCP_Constraint::~Task_Reaction_Force_LCP_Constraint(){}

Task_Reaction_Force_LCP_Constraint::Task_Reaction_Force_LCP_Constraint(WholeBody_Task_List* wb_task_list_in, Contact_List* contact_list_in, int index_in){
}	


void Task_Reaction_Force_LCP_Constraint::Initialization(){
  std::cout << "[Task Reactrion Force LCP Constraint] Initialization called" << std::endl;
  robot_model = RobotModel::GetRobotModel();  
  initialize_Flow_Fupp();	
}

void Task_Reaction_Force_LCP_Constraint::initialize_Flow_Fupp(){
	// ||xddot||^2_2 \cdot ||Fr||^2_2 = 0
	F_low.push_back(0.0);	
	F_upp.push_back(0.0);				
	constraint_size = F_low.size();
}

void Task_Reaction_Force_LCP_Constraint::evaluate_constraint(const int &timestep, WBT_Opt_Variable_List& var_list, std::vector<double>& F_vec){}
void Task_Reaction_Force_LCP_Constraint::evaluate_sparse_gradient(const int &timestep, WBT_Opt_Variable_List& var_list, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG){}

void Task_Reaction_Force_LCP_Constraint::evaluate_sparse_A_matrix(const int &timestep, WBT_Opt_Variable_List& var_list, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA){
	  int n = this->get_constraint_size();//this->num_constraints;
  int m = var_list.get_size_timedependent_vars(); // var_list.get_num_time_dependent_vars
  int T = var_list.total_timesteps; // var_list.get_total_timesteps() Total timestep
  int k = var_list.get_num_keyframe_vars();

  std::cout << "[Contact LCP Constraint] Evaluating A Matrix" << std::endl;

  sejong::Matrix pre_zeroBlock(n, m*timestep); pre_zeroBlock.setZero();
  sejong::Matrix post_zeroBlock(n, m*(T-1-timestep));  post_zeroBlock.setZero();
  sejong::Matrix kf_zeroBlock(n, k);    kf_zeroBlock.setZero();


  // Matrix A = [0, 0, ..., dWBC_i/dx, 0, ..., 0_{T-1-i}, 0] 
  int local_j_offset = 0;
  int i_local = 0;               
  int j_local = local_j_offset;
  // Add pre zero block:
  std::cout << "[Contact LCP Constraint] Constructing Pre Zero block with size: (" << pre_zeroBlock.rows() << "," << pre_zeroBlock.cols() << ")" << std::endl;
  std::cout << "[Contact LCP Constraint] Starting with j index: " << j_local << std::endl;  
  for(size_t i = 0; i < pre_zeroBlock.rows(); i++){
    for(size_t j = 0; j < pre_zeroBlock.cols(); j++){
      A.push_back(pre_zeroBlock(i,j));
      iA.push_back(i_local);
      jA.push_back(j_local);
      j_local++;
    }
    i_local++;
    j_local = local_j_offset; // Reset counter    
  }

  // Add post zero block
  local_j_offset = m*(timestep+1);
  i_local = 0;
  j_local = local_j_offset;
  std::cout << "[Contact LCP Constraint] Constructing Post Zero block with size: (" << post_zeroBlock.rows() << "," << post_zeroBlock.cols() << ")" << std::endl;
  std::cout << "[Contact LCP Constraint] Starting with j index: " << j_local << std::endl;  
  for(size_t i = 0; i < post_zeroBlock.rows(); i++){
    for(size_t j = 0; j < post_zeroBlock.cols(); j++){
      A.push_back(post_zeroBlock(i,j));
      iA.push_back(i_local);
      jA.push_back(j_local);
      j_local++;
    }
    i_local++;
    j_local = local_j_offset; // Reset counter    
  }

  // Add zeros on the keyframe block
  local_j_offset = m*T;
  i_local = 0;
  j_local = local_j_offset;

  std::cout << "[Contact LCP Constraint] Constructing Keyframe block with size: (" << kf_zeroBlock.rows() << "," << kf_zeroBlock.cols() << ")" << std::endl;
  std::cout << "[Contact LCP Constraint] Starting with j index: " << j_local << std::endl;  
  for(size_t i = 0; i < kf_zeroBlock.rows(); i++){
    for(size_t j = 0; j < kf_zeroBlock.cols(); j++){
      A.push_back(kf_zeroBlock(i,j));
      iA.push_back(i_local);
      jA.push_back(j_local);
      j_local++;
    }
    i_local++;
    j_local = local_j_offset; // Reset counter    
  }

}

