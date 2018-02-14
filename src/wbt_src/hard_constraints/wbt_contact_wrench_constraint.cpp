#include <wbt/hard_constraints/wbt_contact_wrench_constraint.hpp>
#include <Utils/utilities.hpp>
#include "valkyrie_definition.h"
#include <wbt/optimization_constants.hpp>

Contact_Wrench_Constraint::Contact_Wrench_Constraint(){
	Initialization();
}

Contact_Wrench_Constraint::Contact_Wrench_Constraint(Contact_List* contact_list_in, int index_in){
  setContact_List(contact_list_in);
  setContact_index(index_in);
  Initialization();
}

Contact_Wrench_Constraint::Contact_Wrench_Constraint(Contact_List* contact_list_in, int index_in, double mu_in, double x_rec_width_in, double y_rec_width_in){
  setContact_List(contact_list_in);
  setContact_index(index_in);  
  mu = mu_in;
  x_rec_width = x_rec_width_in;
  y_rec_width = y_rec_width_in;  
  Initialization();
}

Contact_Wrench_Constraint::~Contact_Wrench_Constraint(){
	std::cout << "Contact Wrench Constraint destructor called" << std::endl;
}

void Contact_Wrench_Constraint::Initialization(){
  std::cout << "[Contact Wrench Constraint] Initialization called!" << std::endl; 
  robot_model = RobotModel::GetRobotModel();  
  initialize_Flow_Fupp();
}

void Contact_Wrench_Constraint::initialize_Flow_Fupp(){
  for (size_t i = 0; i < num_constraints; i++){
    F_low.push_back(0.0);
    F_upp.push_back(OPT_INFINITY);    
  }
  constraint_size = F_low.size();
  std::cout << "Contact Index:" << contact_index << std::endl;
  std::cout << "Contact Name:" <<  contact_list_obj->get_contact(contact_index)->contact_name << std::endl;  


}

void Contact_Wrench_Constraint::setContact_List(Contact_List* contact_list_in){
  contact_list_obj = contact_list_in;
}
void Contact_Wrench_Constraint::setContact_index(int index_in){
  contact_index = index_in;  
}  


void Contact_Wrench_Constraint::evaluate_constraint(const int &timestep, WBT_Opt_Variable_List& var_list, std::vector<double>& F_vec){
  Contact* current_contact = contact_list_obj->get_contact(contact_index);
  int contact_link_id = current_contact->contact_link_id;

  // Get q_states
  sejong::Vector q_state;
  sejong::Vector qdot_state;  
  var_list.get_var_states(timestep, q_state, qdot_state);

  robot_model->UpdateModel(q_state, qdot_state);
  // Update U_int(q);
  UpdateUf(q_state, U_int);

  // Get Fr_states
  sejong::Vector Fr_all;
  var_list.get_var_reaction_forces(timestep, Fr_all);

  int current_contact_size = current_contact->contact_dim;
  if (current_contact_size != 6){
    std::cout << "[Contact Wrench Constraint]: Error! Contact Dimension is not 6." << std::endl;
  }
  // Extract the segment of Reaction Forces corresponding to this contact
  int index_offset = 0;
  for (size_t i = 0; i < contact_index; i++){
    index_offset += contact_list_obj->get_contact(i)->contact_dim;   
  }

  //std::cout << "[Contact Wrench Constraint]: (contact_index, index_offset, contact_dim) = (" << contact_index << "," << index_offset << "," << current_contact_size << ")"  << std::endl;

  sejong::Vector Fr_contact = Fr_all.segment(index_offset, current_contact_size);
  sejong::Vector UFr = U_int*Fr_contact;

/*  sejong::pretty_print(Fr_all, std::cout, "Fr_all");
  sejong::pretty_print(Fr_contact, std::cout, "Fr_contact");*/

  // std::cout << "[Contact Wrench Constraint] UFr Size: " << UFr.size() << std::endl;
  // std::cout << "Should be 17" << std::endl;
  F_vec.clear();

  for (size_t i = 0; i < UFr.size(); i++){
    //std::cout << "UFr[" << i << "] = " << UFr[i] << std::endl;
    F_vec.push_back(UFr[i]);
  }


}

void Contact_Wrench_Constraint::evaluate_sparse_gradient(const int &timestep, WBT_Opt_Variable_List& var_list, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG){
  // Get q_states
  sejong::Vector q_state;
  sejong::Vector qdot_state;  
  var_list.get_var_states(timestep, q_state, qdot_state);

  robot_model->UpdateModel(timestep, q_state, qdot_state);
  // Update U_int(q);
  UpdateUf(q_state, U_int);

  // We have to identify the index where this reaction force starts.
  int index_offset = 0;
  for (size_t i = 0; i < contact_index; i++){
    index_offset += contact_list_obj->get_contact(i)->contact_dim;   
  }  

  // Gradient of Contact Wrench wrt to Fr is U_int
  int m = var_list.get_size_timedependent_vars(); // var_list.get_num_time_dependent_vars
  int local_j_offset = m*timestep + var_list.get_num_q_vars() + var_list.get_num_qdot_vars() + var_list.get_num_xddot_vars() + index_offset;
  //std::cout << "[CWC Constraint]: dF/dFr index j starts at: " << local_j_offset << std::endl; 
  sejong::Matrix F_dFr = U_int;  
  int i_local = 0;               // specify i starting index
  int j_local = local_j_offset;// j = (total_j_size*timestep) + var_states_size + task_accelerations size // specify j starting index
  // Go through F_dFr and push back values to G, iGfun, jGfun 

  //sejong::pretty_print(U_int, std::cout, "U_int");

  for(size_t i = 0; i < F_dFr.rows(); i++){
    for(size_t j = 0; j < F_dFr.cols(); j++){
      //std::cout << "(i,j): " << "(" << i << "," << j << ") = " << F_dFr(i, j) << std::endl;  
      G.push_back(F_dFr(i,j));
      iG.push_back(i_local);
      jG.push_back(j_local);
      j_local++;       
    }
    i_local++;
    j_local = local_j_offset; // Reset counter
  }



}

void Contact_Wrench_Constraint::evaluate_sparse_A_matrix(const int &timestep, WBT_Opt_Variable_List& var_list, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA){
  int n = this->get_constraint_size();
  int m = var_list.get_size_timedependent_vars(); // var_list.get_num_time_dependent_vars
  int T = var_list.total_timesteps; // var_list.get_total_timesteps() Total timestep
  int k = var_list.get_num_keyframe_vars();
  int h_size = var_list.get_num_var_knotpoint_dt();

  std::cout << "[Contact Wrench Constraint] Evaluating A Matrix" << std::endl;

  sejong::Matrix pre_zeroBlock(n, m*timestep); pre_zeroBlock.setZero();
  sejong::Matrix post_zeroBlock(n, m*(T-1-timestep));  post_zeroBlock.setZero();
  sejong::Matrix kf_zeroBlock(n, k);    kf_zeroBlock.setZero();
  sejong::Matrix h_zeroBlock(n, h_size); h_zeroBlock.setZero();


  // Matrix A = [0, 0, ..., dWBC_i/dx, 0, ..., 0_{T-1-i}, 0] 
  int local_j_offset = 0;
  int i_local = 0;               
  int j_local = local_j_offset;
  // Add pre zero block:
  std::cout << "[Contact Wrench Constraint] Constructing Pre Zero block with size: (" << pre_zeroBlock.rows() << "," << pre_zeroBlock.cols() << ")" << std::endl;
  std::cout << "[Contact Wrench Constraint] Starting with j index: " << j_local << std::endl;  
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
  std::cout << "[Contact Wrench Constraint] Constructing Post Zero block with size: (" << post_zeroBlock.rows() << "," << post_zeroBlock.cols() << ")" << std::endl;
  std::cout << "[Contact Wrench Constraint] Starting with j index: " << j_local << std::endl;  
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

  std::cout << "[Contact Wrench Constraint] Constructing Keyframe block with size: (" << kf_zeroBlock.rows() << "," << kf_zeroBlock.cols() << ")" << std::endl;
  std::cout << "[Contact Wrench Constraint] Starting with j index: " << j_local << std::endl;  
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


  // Add zeros on the h_dt block
  local_j_offset = m*T + k;
  i_local = 0;
  j_local = local_j_offset;  
  std::cout << "[Contact Wrench  Constraint] Constructing h_dt block with size: (" << kf_zeroBlock.rows() << "," << kf_zeroBlock.cols() << ")" << std::endl;
  std::cout << "[Contact Wrench  Constraint] Starting with j index: " << j_local << std::endl;  
  for(size_t i = 0; i < h_zeroBlock.rows(); i++){
    for(size_t j = 0; j < h_zeroBlock.cols(); j++){
      A.push_back(h_zeroBlock(i,j));
      iA.push_back(i_local);
      jA.push_back(j_local);
      j_local++;
    }
    i_local++;
    j_local = local_j_offset; // Reset counter    
  }  
  


}

void Contact_Wrench_Constraint::UpdateUf(const sejong::Vector &q_state, sejong::Matrix &Uf){
  Contact* current_contact = contact_list_obj->get_contact(contact_index);
  int contact_link_id = current_contact->contact_link_id;

  int size_u(17);
  int dim_contact_ = 6;
  Uf = sejong::Matrix::Zero(size_u, dim_contact_);

  sejong::Matrix U;
  setU(x_rec_width, y_rec_width, mu, U);
  Eigen::Quaternion<double> quat_tmp;

  robot_model->getOrientation(q_state, contact_link_id, quat_tmp);
  Eigen::Matrix3d R_link_mtx(quat_tmp);
  sejong::Matrix R_link(6,6); R_link.setZero();
  R_link.block(0,0, 3,3) = R_link_mtx.transpose();
  R_link.block(3,3, 3,3) = R_link_mtx.transpose();

  Uf.block(0,0, size_u, 6) = U * R_link;
//  return true;

}

void Contact_Wrench_Constraint::setU(const double x, const double y, const double mu, sejong::Matrix & U){
  U = sejong::Matrix::Zero(17, 6);

  U(0, 5) = 1.;

  U(1, 3) = 1.; U(1, 5) = mu;
  U(2, 3) = -1.; U(2, 5) = mu;

  U(3, 4) = 1.; U(3, 5) = mu;
  U(4, 4) = -1.; U(4, 5) = mu;

  U(5, 0) = 1.; U(5, 5) = y;
  U(6, 0) = -1.; U(6, 5) = y;

  U(7, 1) = 1.; U(7, 5) = x;
  U(8, 1) = -1.; U(8, 5) = x;

  // Tau
  U(9, 0) = -mu; U(9, 1) = -mu; U(9, 2) = 1;
  U(9, 3) = y;   U(9, 4) = x;   U(9, 5) = (x + y)*mu;

  U(10, 0) = -mu; U(10, 1) = mu; U(10, 2) = 1;
  U(10, 3) = y;   U(10, 4) = -x; U(10, 5) = (x + y)*mu;

  U(11, 0) = mu; U(11, 1) = -mu; U(11, 2) = 1;
  U(11, 3) = -y; U(11, 4) = x;   U(11, 5) = (x + y)*mu;

  U(12, 0) = mu; U(12, 1) = mu; U(12, 2) = 1;
  U(12, 3) = -y; U(12, 4) = -x; U(12, 5) = (x + y)*mu;
  /////////////////////////////////////////////////
  U(13, 0) = -mu; U(13, 1) = -mu; U(13, 2) = -1;
  U(13, 3) = -y;  U(13, 4) = -x;  U(13, 5) = (x + y)*mu;

  U(14, 0) = -mu; U(14, 1) = mu; U(14, 2) = -1;
  U(14, 3) = -y;  U(14, 4) = x;  U(14, 5) = (x + y)*mu;

  U(15, 0) = mu; U(15, 1) = -mu; U(15, 2) = -1;
  U(15, 3) = y;  U(15, 4) = -x;  U(15, 5) = (x + y)*mu;

  U(16, 0) = mu; U(16, 1) = mu; U(16, 2) = -1;
  U(16, 3) = y;  U(16, 4) = x;  U(16, 5) = (x + y)*mu;
}