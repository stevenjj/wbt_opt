#include <wbt/hard_constraints/wbt_wholebody_controller_constraint.hpp>
#include <Utils/utilities.hpp>
#include "valkyrie_definition.h"

Wholebody_Controller_Constraint::Wholebody_Controller_Constraint(){
	Initialization();
}

Wholebody_Controller_Constraint::Wholebody_Controller_Constraint(WholeBody_Task_List* wb_task_list_input){
	Initialization();
	set_task_list(wb_task_list_input);
}

Wholebody_Controller_Constraint::Wholebody_Controller_Constraint(WholeBody_Task_List* wb_task_list_input, Contact_List* contact_list_input){
	Initialization();
	set_task_list(wb_task_list_input);
	set_contact_list(contact_list_input);
}

Wholebody_Controller_Constraint::~Wholebody_Controller_Constraint(){
	std::cout << "WCC destructor called" << std::endl;
}


void Wholebody_Controller_Constraint::Initialization(){
  robot_model = RobotModel::GetRobotModel();	
  A_int.resize(NUM_QDOT, NUM_QDOT);

  Sv.resize(NUM_VIRTUAL, NUM_QDOT);
  Sa.resize(NUM_ACT_JOINT, NUM_QDOT);
  Sv.setZero();
  Sa.setZero();

  Sv.block(0,0, NUM_VIRTUAL, NUM_VIRTUAL) = sejong::Matrix::Identity(NUM_VIRTUAL, NUM_VIRTUAL);
  Sa.block(0, NUM_VIRTUAL, NUM_ACT_JOINT, NUM_ACT_JOINT) = sejong::Matrix::Identity(NUM_ACT_JOINT, NUM_ACT_JOINT);
  torque_limit = 1800;

  initialize_Flow_Fupp();
}

void Wholebody_Controller_Constraint::initialize_Flow_Fupp(){
  // WBC Virtual Torque Constraints
  for(size_t i = 0; i < NUM_VIRTUAL; i++){
    F_low.push_back(0.0);
    F_upp.push_back(0.0);        
  }
  // WBC Torque Limit Constraints
  for (size_t i = 0; i < NUM_QDOT-NUM_VIRTUAL; i++){
    F_low.push_back(-torque_limit);
    F_upp.push_back(torque_limit);    
  }
  constraint_size = F_low.size();
  //std::cout << "Size of (Flow, Fupp): (" << F_low.size() << ", " << F_upp.size() << ")" << std::endl;
}

void Wholebody_Controller_Constraint::set_task_list(WholeBody_Task_List* wb_task_list_input){
	std::cout << "[WBC Constraint] Processing Task List" << std::endl;

	wb_task_list = wb_task_list_input;	
	task_dim = 0;
	for(size_t i = 0; i < wb_task_list->get_size(); i++){
		task_dim += wb_task_list->get_task(i)->task_dim;
	}

	std::cout << "[WBC Constraint] Task List Processed" << std::endl;
	std::cout << "[WBC Constraint] Task List size: " << wb_task_list->get_size() << std::endl;
	std::cout << "[WBC Constraint] Task Dimension size: " << task_dim << std::endl;
}



void Wholebody_Controller_Constraint::set_contact_list(Contact_List* contact_list_input){
	std::cout << "[WBC Constraint] Processing Contact List" << std::endl;

	contact_list = contact_list_input;

  contact_dim = 0;
  for(size_t i = 0; i < contact_list->get_size(); i++){
    contact_dim += contact_list->get_contact(i)->contact_dim;
  }  

	std::cout << "[WBC Constraint] Contact List Processed" << std::endl;
  std::cout << "[WBC Constraint] Contact List size: " << contact_list->get_size() << std::endl;
	std::cout << "[WBC Constraint] Contact Dimension: " << contact_dim << std::endl;
}

void Wholebody_Controller_Constraint::test_function(){
	std::cout << "[WBC Constraint] Test function called" << std::endl;
}

void Wholebody_Controller_Constraint::test_function2(const sejong::Vector &q, const sejong::Vector &qdot, sejong::Matrix &B_out, sejong::Vector &c_out){
	get_Jc(q, Jc_int);
	sejong::pretty_print(Jc_int, std::cout, "WBDC: Jc_int");
}


void Wholebody_Controller_Constraint::UpdateModel(const sejong::Vector &q, const sejong::Vector &qdot,
                                                  sejong::Matrix &A_out, sejong::Vector &grav_out, sejong::Vector &cori_out){
  A_out.resize(NUM_QDOT, NUM_QDOT);
  grav_out.resize(NUM_QDOT, 1);
  cori_out.resize(NUM_QDOT, 1);
  A_out.setZero();
  grav_out.setZero();
  cori_out.setZero();  

  robot_model->UpdateModel(q, qdot);
  robot_model->getMassInertia(A_out); 
  robot_model->getGravity(grav_out);  
  robot_model->getCoriolis(cori_out); 
}

void Wholebody_Controller_Constraint::UpdateModel(const int &timestep, const sejong::Vector &q, const sejong::Vector &qdot,
                      sejong::Matrix &A_out, sejong::Vector &grav_out, sejong::Vector &cori_out){
  A_out.resize(NUM_QDOT, NUM_QDOT);
  grav_out.resize(NUM_QDOT, 1);
  cori_out.resize(NUM_QDOT, 1);
  A_out.setZero();
  grav_out.setZero();
  cori_out.setZero();  

  robot_model->UpdateModel(timestep, q, qdot);
  robot_model->getMassInertia(A_out); 
  robot_model->getGravity(grav_out);  
  robot_model->getCoriolis(cori_out);     
}

void Wholebody_Controller_Constraint::getB_c(const sejong::Vector &q, const sejong::Vector &qdot, sejong::Matrix &B_out, sejong::Vector &c_out){
  sejong::Matrix B;
  sejong::Vector c;  

  sejong::Matrix Ainv;
  sejong::Matrix Jt, JtPre;
  sejong::Matrix Jt_inv, JtPre_inv;
  sejong::Vector JtDotQdot;
  sejong::Vector xddot;
  sejong::Matrix Npre;
  sejong::Matrix I_JtPreInv_Jt;
  Task* task = wb_task_list->get_task(0);

  int tot_task_size(0);

  //robot_model->UpdateModel(q, qdot); // Update Model. This call should be moved so that it's only called once.
  robot_model->getInverseMassInertia(Ainv);
  task->getTaskJacobian(q, Jt);
  task->getTaskJacobianDotQdot(q, qdot, JtDotQdot);

  _WeightedInverse(Jt, Ainv, Jt_inv);
  B = Jt_inv;
  c = Jt_inv * JtDotQdot;


  Npre = sejong::Matrix::Identity(NUM_QDOT, NUM_QDOT) - Jt_inv * Jt;
  tot_task_size += task->task_dim;



  for(int i(1); i<wb_task_list->get_size(); ++i){
    // Obtaining Task
    task = wb_task_list->get_task(i);

    task->getTaskJacobian(q, Jt);
    task->getTaskJacobianDotQdot(q, qdot, JtDotQdot);   
    JtPre = Jt * Npre;
    _WeightedInverse(JtPre, Ainv, JtPre_inv);
    I_JtPreInv_Jt = sejong::Matrix::Identity(NUM_QDOT, NUM_QDOT) - JtPre_inv * Jt;

    // B matrix building
    B.conservativeResize(NUM_QDOT, tot_task_size + task->task_dim);
    B.block(0, 0, NUM_QDOT, tot_task_size) =
      I_JtPreInv_Jt * B.block(0, 0, NUM_QDOT, tot_task_size);
    B.block(0, tot_task_size, NUM_QDOT, task->task_dim) = JtPre_inv;
    // c vector building
    c = I_JtPreInv_Jt * c - JtPre_inv * JtDotQdot;

    // Build for Next
    Npre = Npre * ( sejong::Matrix::Identity(NUM_QDOT, NUM_QDOT) - JtPre_inv * JtPre);
    tot_task_size += task->task_dim;

  }

/*  sejong::pretty_print(B, std::cout, "WBDC: B");
  sejong::pretty_print(c, std::cout, "WBDC: c");*/

  B_out = B;
  c_out = c;
}

// Gets the contact Jacobian Jc
void Wholebody_Controller_Constraint::get_Jc(const sejong::Vector &q, sejong::Matrix &Jc_out){
  Contact* contact;
  sejong::Matrix Jc;
  sejong::Matrix Jtmp;
  int total_contact_dim = 0;
  for(int i = 0; i < contact_list->get_size(); i++){ 	
  	contact = contact_list->get_contact(i);
  	contact->getContactJacobian(q, Jtmp); 

  	Jc.conservativeResize(total_contact_dim + contact->contact_dim, NUM_QDOT);
  	Jc.block(total_contact_dim, 0, contact->contact_dim, NUM_QDOT) = Jtmp;

  	total_contact_dim += contact->contact_dim;
  }

  Jc_out = Jc;
}

void Wholebody_Controller_Constraint::evaluate_constraint(const int &timestep, WBT_Opt_Variable_List& var_list, std::vector<double>& F_vec){
  sejong::Vector q_state;
  sejong::Vector qdot_state;
  sejong::Vector xddot_des;
  sejong::Vector Fr;

  var_list.get_var_states(timestep, q_state, qdot_state);
  var_list.get_task_accelerations(timestep, xddot_des);
  var_list.get_var_reaction_forces(timestep, Fr);

  //sejong::pretty_print(Fr, std::cout, "Fr");

  //std::cout << "    WBC evaluating constraint" << std::endl;

  sejong::Vector g(NUM_QDOT, 1);
  sejong::Vector b(NUM_QDOT, 1);
  UpdateModel(q_state, qdot_state, A_int, g, b);
  last_timestep_model_update = timestep;

  getB_c(q_state, qdot_state, B_int, c_int);
  get_Jc(q_state, Jc_int);    

  sejong::Vector qddot_des = (B_int*xddot_des + c_int);
  sejong::Vector WB_des = A_int*qddot_des + b + g - Jc_int.transpose()*Fr; // Aqddot_des + b + g - J^T_c Fr = [0, tau]^T; 

  sejong::Vector WBC_virtual_constraints(NUM_VIRTUAL);
  sejong::Vector tau_constraints(NUM_ACT_JOINT);
  
  WBC_virtual_constraints = Sv*(WB_des);
  tau_constraints = Sa*WB_des;

  // sejong::pretty_print(q_state, std::cout, "q_state");
  // sejong::pretty_print(qdot_state, std::cout, "qdot_state");

  // sejong::pretty_print(WB_des, std::cout, "WB_des");
  // sejong::pretty_print(WBC_virtual_constraints, std::cout, "WBC_virtual_constraints");  
  // sejong::pretty_print(tau_constraints, std::cout, "tau_constraints");    

  F_vec.clear();
  // Populate F_vec, order matters here:
  for(size_t i = 0; i < WBC_virtual_constraints.size(); i++){
    F_vec.push_back(WBC_virtual_constraints[i]);
  }  
  for(size_t i = 0; i < tau_constraints.size(); i++){
    F_vec.push_back(tau_constraints[i]);
  }    
}


void Wholebody_Controller_Constraint::evaluate_sparse_A_matrix(const int &timestep, WBT_Opt_Variable_List& var_list, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA){
  int n = this->get_constraint_size();
  int m = var_list.get_size_timedependent_vars(); // var_list.get_num_time_dependent_vars
  int T = var_list.total_timesteps; // var_list.get_total_timesteps() Total timestep
  int k = var_list.get_num_keyframe_vars();

  std::cout << "[WBC Constraint] Evaluating A Matrix" << std::endl;

  sejong::Matrix pre_zeroBlock(n, m*timestep); pre_zeroBlock.setZero();
  sejong::Matrix post_zeroBlock(n, m*(T-1-timestep));  post_zeroBlock.setZero();
  sejong::Matrix kf_zeroBlock(n, k);    kf_zeroBlock.setZero();


  // Matrix A = [0, 0, ..., dWBC_i/dx, 0, ..., 0_{T-1-i}, 0] 
  int local_j_offset = 0;
  int i_local = 0;               
  int j_local = local_j_offset;
  // Add pre zero block:
  std::cout << "[WBC Constraint] Constructing Pre Zero block with size: (" << pre_zeroBlock.rows() << "," << pre_zeroBlock.cols() << ")" << std::endl;
  std::cout << "[WBC Constraint] Starting with j index: " << j_local << std::endl;  
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
  std::cout << "[WBC Constraint] Constructing Post Zero block with size: (" << post_zeroBlock.rows() << "," << post_zeroBlock.cols() << ")" << std::endl;
  std::cout << "[WBC Constraint] Starting with j index: " << j_local << std::endl;  
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

  std::cout << "[WBC Constraint] Constructing Keyframe block with size: (" << kf_zeroBlock.rows() << "," << kf_zeroBlock.cols() << ")" << std::endl;
  std::cout << "[WBC Constraint] Starting with j index: " << j_local << std::endl;  
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



void Wholebody_Controller_Constraint::evaluate_sparse_gradient(const int &timestep, WBT_Opt_Variable_List& var_list, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG){
  // std::cout << "[WBC Constraint] Sparse Gradient Called" << std::endl;
  // std::cout << "[WBC Constraint]: Current Timestep " << timestep << " Last Timestep That Model was updated:" << last_timestep_model_update << std::endl;

  if (timestep != last_timestep_model_update){
    //std::cout << "    Timestep does not match. Will update model" << std::endl;    
    sejong::Vector q_state;
    sejong::Vector qdot_state; 
    var_list.get_var_states(timestep, q_state, qdot_state);    
    sejong::Vector g(NUM_QDOT, 1);
    sejong::Vector b(NUM_QDOT, 1);
    UpdateModel(q_state, qdot_state, A_int, g, b);       
    last_timestep_model_update = timestep;
    getB_c(q_state, qdot_state, B_int, c_int);
    get_Jc(q_state, Jc_int);    
  }

  int m = var_list.get_size_timedependent_vars(); // var_list.get_num_time_dependent_vars


  // Assign Known Elements ----------------------------------------------------------------------------------------------------------
  //std::cout << "Size of Task Acceleration Vars: " << var_list.get_num_xddot_vars() <<  std::endl;

  // Gradient of WBC wrt to Fr is -J_c^T
  int local_j_offset = m*timestep + var_list.get_num_q_vars() + var_list.get_num_qdot_vars() + var_list.get_num_xddot_vars();
  //std::cout << "[WBC Constraint]: dF/dFr index j starts at: " << local_j_offset << std::endl; 
  sejong::Matrix F_dFr = -Jc_int.transpose();  
  int i_local = 0;               // specify i starting index
  int j_local = local_j_offset;// j = (total_j_size*timestep) + var_states_size + task_accelerations size // specify j starting index
  // Go through F_dFr and push back values to G, iGfun, jGfun 

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


  // Gradient of WBC wrt to xddot is A*B(q)
  // i = 0               // specify i starting index
  // j = (total_j_size*timestep) var_states_size // specify j starting index
  // Go through F_dxddot and push back values to G, iGfun, jGfun

  sejong::Matrix F_dxddot = A_int*B_int;
  local_j_offset = m*timestep + var_list.get_num_q_vars() + var_list.get_num_qdot_vars();
  i_local = 0;
  j_local = local_j_offset;
  for(size_t i = 0; i < F_dxddot.rows(); i++){
    for(size_t j = 0; j < F_dxddot.cols(); j++){
      //std::cout << "(i,j): " << "(" << i << "," << j << ") = " << F_dxddot(i, j) << std::endl;  
      G.push_back(F_dxddot(i,j));
      iG.push_back(i_local);
      jG.push_back(j_local);
      j_local++;       
    }
    i_local++;
    j_local = local_j_offset; // Reset counter
  }



  // sejong::pretty_print(F_dxddot, std::cout, "F_dxddot");  
  //sejong::pretty_print(F_dFr, std::cout, "F_dFr");    

}


  // Assign 0's on elements that have no impact (key frames)
  // i = 0
  // j = (total_j_size*timestep) + var_states_size + task_acceleration_size + reaction_force_size
  // sejong::Matrix zeroMat(this->get_constraint_size(), var_keyframes_size)
