#include <wbt/optimization_problems/wbt_opt_problem_wbdc.hpp>
#include <Utils/utilities.hpp>

#include <wbt/tasks/wbt_task_leftfoot.hpp>
#include <wbt/tasks/wbt_task_rightfoot.hpp>
#include <wbt/contacts/wbt_contact_leftfoot.hpp>
#include <wbt/contacts/wbt_contact_rightfoot.hpp>

//#define USE_OBJECTIVE_GRADIENT

WBDC_Opt::WBDC_Opt(){
  this->problem_name = "WBDC Optimization Problem";  
	robot_model = RobotModel::GetRobotModel();

	robot_q_init.resize(NUM_Q); 
	robot_qdot_init.resize(NUM_QDOT);	

	robot_q_init.setZero(); 
	robot_qdot_init.setZero();

  ptr_wbc_constraint = new Wholebody_Controller_Constraint();
  Initialization();
}

WBDC_Opt::~WBDC_Opt(){
  std::cout << "[WBDC OPT] Destructor Called" << std::endl;
}

void WBDC_Opt::Initialization(){
	std::cout << "[WBDC_Opt] Initialization Called" << std::endl;
 	total_timesteps = 2;
  initialize_starting_configuration();
	initialize_task_list();
	initialize_contact_list();
  initialize_td_constraint_list();  
  initialize_opt_vars();
  initialize_objective_func();


  std::vector<double> F_low_test;
  std::vector<double> F_upp_test;   
  get_F_bounds(F_low_test, F_upp_test);

}

void WBDC_Opt::initialize_starting_configuration(){
 // Set Virtual Joints
  // x_pos
  robot_q_init[0] = 0.0;
  // y_pos
  robot_q_init[1] = 0.0;
  // z_pos
  robot_q_init[2] = 1.14; //1.135; //1.131; 
  robot_q_init[NUM_QDOT] = 1.0; // Pelvis Quaternion w = 1.0


  // Initialize Joints
  robot_q_init[NUM_VIRTUAL + SJJointID::leftHipPitch] = -0.3; //r_joint_[r_joint_idx_map_.find("leftHipPitch"  )->second]->m_State.m_rValue[0] = -0.3;
  robot_q_init[NUM_VIRTUAL + SJJointID::rightHipPitch] = -0.3;  //r_joint_[r_joint_idx_map_.find("rightHipPitch" )->second]->m_State.m_rValue[0] = -0.3;
  robot_q_init[NUM_VIRTUAL + SJJointID::leftKneePitch] = 0.6;  //r_joint_[r_joint_idx_map_.find("leftKneePitch" )->second]->m_State.m_rValue[0] = 0.6;
  robot_q_init[NUM_VIRTUAL + SJJointID::rightKneePitch] = 0.6;//r_joint_[r_joint_idx_map_.find("rightKneePitch")->second]->m_State.m_rValue[0] = 0.6;
  robot_q_init[NUM_VIRTUAL + SJJointID::leftAnklePitch] = -0.3; //r_joint_[r_joint_idx_map_.find("leftAnklePitch")->second]->m_State.m_rValue[0] = -0.3;
  robot_q_init[NUM_VIRTUAL + SJJointID::rightAnklePitch] = -0.3; //r_joint_[r_joint_idx_map_.find("rightAnklePitch")->second]->m_State.m_rValue[0] = -0.3;

  robot_q_init[NUM_VIRTUAL + SJJointID::rightShoulderPitch] = 0.2; //r_joint_[r_joint_idx_map_.find("rightShoulderPitch")->second]->m_State.m_rValue[0] = 0.2;
  robot_q_init[NUM_VIRTUAL + SJJointID::rightShoulderRoll] = 1.1;  //r_joint_[r_joint_idx_map_.find("rightShoulderRoll" )->second]->m_State.m_rValue[0] = 1.1;
  robot_q_init[NUM_VIRTUAL + SJJointID::rightElbowPitch] = 0.4;  //r_joint_[r_joint_idx_map_.find("rightElbowPitch"   )->second]->m_State.m_rValue[0] = 0.4;
  robot_q_init[NUM_VIRTUAL + SJJointID::rightForearmYaw] = 1.5;  //r_joint_[r_joint_idx_map_.find("rightForearmYaw" )->second]->m_State.m_rValue[0] = 1.5;

  robot_q_init[NUM_VIRTUAL + SJJointID::leftShoulderPitch] = -0.2; //r_joint_[r_joint_idx_map_.find("rightShoulderPitch")->second]->m_State.m_rValue[0] = 0.2;
  robot_q_init[NUM_VIRTUAL + SJJointID::leftShoulderRoll] = -1.1;  //r_joint_[r_joint_idx_map_.find("rightShoulderRoll" )->second]->m_State.m_rValue[0] = 1.1;
  robot_q_init[NUM_VIRTUAL + SJJointID::leftElbowPitch] = -0.4;//0.4;  //r_joint_[r_joint_idx_map_.find("rightElbowPitch"   )->second]->m_State.m_rValue[0] = 0.4;
  robot_q_init[NUM_VIRTUAL + SJJointID::leftForearmYaw] = 1.5;  //r_joint_[r_joint_idx_map_.find("rightForearmYaw" )->second]->m_State.m_rValue[0] = 1.5;	

  std::cout << "[WBDC_Opt] Robot Starting State Initialized" << std::endl;
  //sejong::pretty_print(robot_q_init, std::cout, "Q init");
}

void WBDC_Opt::initialize_task_list(){
/*	wb_task_list.append_task(new LeftFoot_Task());
  std::cout << "[WBDC_Opt] Task List Initialized" << std::endl;  
  wbc_constraint.set_task_list(&wb_task_list);*/
  wb_task_list.append_task(new LeftFoot_Task());
  wb_task_list.append_task(new RightFoot_Task());  
}

void WBDC_Opt::initialize_contact_list(){
  contact_list.append_contact(new LeftFoot_Contact());
  contact_list.append_contact(new RightFoot_Contact());
  std::cout << "[WBDC_Opt] Contact List Initialized" << std::endl;  
}

void WBDC_Opt::initialize_td_constraint_list(){
  std::cout << "[WBDC_Opt] Initializing WBC Constraints" << std::endl;

  ptr_wbc_constraint->set_task_list(&wb_task_list);
  ptr_wbc_constraint->set_contact_list(&contact_list);  
//  td_constraint_list.append_constraint(new Wholebody_Controller_Constraint(&wb_task_list, &contact_list));
  td_constraint_list.append_constraint(ptr_wbc_constraint);  

  int left_foot_index = 0;
  int right_foot_index = 1;  

  td_constraint_list.append_constraint(new Contact_Wrench_Constraint(&contact_list, left_foot_index, 0.8, 0.1, 0.05));  
  td_constraint_list.append_constraint(new Contact_Wrench_Constraint(&contact_list, right_foot_index, 0.8, 0.1, 0.05));  

  td_constraint_list.append_constraint(new Contact_LCP_Constraint(&contact_list, left_foot_index));
  td_constraint_list.append_constraint(new Contact_LCP_Constraint(&contact_list, right_foot_index));      
    


  // Test WBC B and c matrix construction
/*  sejong::Matrix B_test;
  sejong::Vector c_test;  
  td_constraint_list.get_constraint(0)->test_function2(robot_q_init, robot_qdot_init, B_test, c_test);*/

}

// Main class implementation
void WBDC_Opt::initialize_opt_vars(){
  // For each timestep:
    // timestep = 0 is a special case
    // Populate wbt_opt_variable_list to initialize x_value, xlow, xupp

  std::cout << "[WBDC_OPT] Initializing Optimization Variables" << std::endl;

  for(size_t i = 0; i < total_timesteps; i++){
    // -------------------------------------------------------------------------------------------------------------------
    // Robot State Initialization
    // If timestep is 0. The robot states(q qdot) are set to the initial configuration.
    if (i == 0){
      for(size_t j = 0; j < NUM_Q; j++){
        WBT_Opt_Variable* q_var = new WBT_Opt_Variable("q_state", VAR_TYPE_Q, i, robot_q_init[j], robot_q_init[j] - OPT_ZERO_EPS, robot_q_init[j] + OPT_ZERO_EPS);
        opt_var_list.append_variable(q_var);
      }
      for(size_t j = 0; j < NUM_QDOT; j++){
        WBT_Opt_Variable* qdot_var = new WBT_Opt_Variable("qdot_state", VAR_TYPE_QDOT, i, robot_qdot_init[j], robot_qdot_init[j] - OPT_ZERO_EPS, robot_qdot_init[j] + OPT_ZERO_EPS);
        opt_var_list.append_variable(qdot_var);
      }
    }else{
      // For all future configuration evolutions, apply joint limit constraints. For now, let's set joints to be unbounded.
      for(size_t j = 0; j < NUM_Q; j++){
        WBT_Opt_Variable* q_var = new WBT_Opt_Variable("q_state", VAR_TYPE_Q, i, robot_q_init[j], -OPT_INFINITY, OPT_INFINITY);
        opt_var_list.append_variable(q_var);
      }
      for(size_t j = 0; j < NUM_QDOT; j++){
        WBT_Opt_Variable* qdot_var = new WBT_Opt_Variable("qdot_state", VAR_TYPE_QDOT, i, robot_qdot_init[j], -OPT_INFINITY, OPT_INFINITY);
        opt_var_list.append_variable(qdot_var);      
      }
    }
    //---------------------------------------------------------------------------------------------------------------------

    //---------------------------------------------------------------------------------------------------------------------  
    // Task Acceleration Initialization   
    for(size_t j = 0; j < ptr_wbc_constraint->task_dim; j++){
//        WBT_Opt_Variable* xddot_var = new WBT_Opt_Variable("xddot", VAR_TYPE_TA, i, 0.0, -OPT_INFINITY, OPT_INFINITY);
        WBT_Opt_Variable* xddot_var = new WBT_Opt_Variable("xddot", VAR_TYPE_TA, i, 0.0, -0.0001, 0.0001);
        opt_var_list.append_variable(xddot_var);      
    }
    //---------------------------------------------------------------------------------------------------------------------

    //---------------------------------------------------------------------------------------------------------------------  
    // Reaction Force Initialization   
    for(size_t j = 0; j < ptr_wbc_constraint->contact_dim; j++){
        WBT_Opt_Variable* Fr_var = new WBT_Opt_Variable("Fr", VAR_TYPE_FR, i, 0.0, -OPT_INFINITY, OPT_INFINITY);
        opt_var_list.append_variable(Fr_var);      
    }
    //---------------------------------------------------------------------------------------------------------------------    

  }
  // Key Frame List Initialization
  // Insert initialization here
  //

  // Specify total timesteps
  opt_var_list.total_timesteps = total_timesteps;
  std::cout << "[WBDC_OPT] Computing Size of Time Dependent Variables " << std::endl;

  // This must be computed...
  opt_var_list.compute_size_time_dep_vars();

  // ----------- End Initialization

  std::cout << "[WBDC_OPT] Total Timesteps: " << total_timesteps << std::endl;
  std::cout << "[WBDC_OPT] Total number of optimization variables: " << opt_var_list.get_size() << std::endl;
  std::cout << "[WBDC_OPT] Total number of time dependent optimization variables: " << opt_var_list.get_size_timedependent_vars() << std::endl;

  sejong::Vector q_state_test;
  sejong::Vector qdot_state_test;  
  sejong::Vector xddot_test;
  sejong::Vector Fr_test;
  opt_var_list.get_var_states(0, q_state_test, qdot_state_test);
  opt_var_list.get_task_accelerations(0, xddot_test);  
  opt_var_list.get_var_reaction_forces(0, Fr_test);
}

void WBDC_Opt::get_init_opt_vars(std::vector<double> &x_vars){ 

  for(size_t i = 0; i < opt_var_list.get_size(); i++){
    x_vars.push_back(opt_var_list.get_opt_variable(i)->value);
  }


}

void WBDC_Opt::get_opt_vars_bounds(std::vector<double> &x_low, std::vector<double> &x_upp){
  for(size_t i = 0; i < opt_var_list.get_size(); i++){
    x_low.push_back(opt_var_list.get_opt_variable(i)->l_bound);
    x_upp.push_back(opt_var_list.get_opt_variable(i)->u_bound);    
  }

}

void WBDC_Opt::update_opt_vars(std::vector<double> &x_vars){ 
  opt_var_list.update_x(x_vars);
}

void WBDC_Opt::get_current_opt_vars(std::vector<double> &x_vars_out){
  opt_var_list.populate_x(x_vars_out);
}

void WBDC_Opt::get_F_bounds(std::vector<double> &F_low, std::vector<double> &F_upp){
  // Initialize Bounds for Time Dependent Constraints
  for(int timestep = 0; timestep < total_timesteps; timestep++){

    for(size_t i = 0; i < td_constraint_list.get_size(); i++){
      for(size_t j = 0; j < td_constraint_list.get_constraint(i)->F_low.size(); j++ ){
        F_low.push_back(td_constraint_list.get_constraint(i)->F_low[j]);
      }
      for(size_t j = 0; j < td_constraint_list.get_constraint(i)->F_upp.size(); j++ ){
        F_upp.push_back(td_constraint_list.get_constraint(i)->F_upp[j]);
      }
    }

  }

  // Initialize Bounds for Time Independent Constraints
  // Code Here


  // Initialize Bounds for the Objective Function
  F_low.push_back(objective_function.F_low);
  F_upp.push_back(objective_function.F_upp);  


  // Debug Statements
/*  for (size_t i = 0; i < F_low.size(); i++){
    std::cout << "F_low[i] = " << F_low[i] << std::endl;
  }

  for (size_t i = 0; i < F_upp.size(); i++){
    std::cout << "F_upp[i] = " << F_upp[i] << std::endl;
  }*/

  // Add Optimization Bounds

  // For each timestep:
    // Populate constraint function list

    // Given Task List
    // WBC constraint bounds

    // Given Contact List, create:
      // Contact and Task LCP Constraints
      // Friction Constraints bounds

    // Construct Time Integration Constraint bounds
}

void WBDC_Opt::get_F_obj_Row(int &obj_row){
  obj_row = objective_function.objective_function_index;
}

void WBDC_Opt::initialize_objective_func(){
  std::cout << "[WBDC Opt] Initializing Objective Function" << std::endl;
  objective_function.set_var_list(opt_var_list);

  // Obj Func Row = m*T + num hard keyframes

  // |F_td| = num of time dependent constraint functions
  // T = total timesteps
  objective_function.objective_function_index = td_constraint_list.get_num_constraint_funcs()*total_timesteps;
  // If there are none time-dependent constraints, we have to add them here

  std::cout << "[WBDC Opt] Objective Function has index: " << objective_function.objective_function_index << std::endl;

  objective_function.setQ_vals(5,5, 0.001);
  objective_function.setQ_vals(11,11, 0.001);  
}




void WBDC_Opt::compute_F_constraints(std::vector<double> &F_eval){
  // Update var_list

  // We know the size of F.
  // Compute F(timestep, wbt_opt_var_list)
  std::vector<double> F_vec_const;
  //std::cout << "[WBDC OPT] Computing F Constraints" << std::endl;

  // Compute Timestep Dependent Constraints
  for(int timestep = 0; timestep < total_timesteps; timestep++){
    for(int i = 0; i < td_constraint_list.get_size(); i++){
      F_vec_const.clear();
      td_constraint_list.get_constraint(i)->evaluate_constraint(timestep, opt_var_list, F_vec_const);
      for(int j = 0; j < F_vec_const.size(); j++){
        //std::cout << "F_vec_const[j] = " << F_vec_const[j] << std::endl;
        // Add to F_eval
        F_eval.push_back(F_vec_const[j]);
      }
    }
  }

  // Compute Timestep Independent Constraints
  // Code here

  // Debug statement  
  /*  for(int j = 0; j < F_eval.size(); j++){
      std::cout << "F_eval[" << j << "] = " << F_eval[j] << std::endl;
    }*/

}

void WBDC_Opt::compute_F_objective_function(double &result_out){
  //std::cout << "[WBDC Opt] Evaluating Objective Function" << std::endl; 
  objective_function.evaluate_objective_function(opt_var_list, result_out);
}


void WBDC_Opt::compute_F(std::vector<double> &F_eval){
  compute_F_constraints(F_eval);
  double cost = 0.0;
  compute_F_objective_function(cost);
  F_eval.push_back(cost);
}

void WBDC_Opt::compute_A(std::vector<double> &A_eval, std::vector<int> &iAfun, std::vector<int> &jAvar, int &neA){
  std::vector<double> A_local;
  std::vector<int> iAfun_local;
  std::vector<int> jAvar_local;

  int constraint_index = -1;
  int iAfun_absolute_start = -1;
  int iAfun_absolute_index = 0;

  int num_time_dependent_constraints_funcs = td_constraint_list.get_num_constraint_funcs(); // find the length of time-dependent constraint functions, F
  for(int timestep = 0; timestep < total_timesteps; timestep++){
    // Evaluate Known Constraint Gradient Elements
    for(int i = 0; i < td_constraint_list.get_size(); i++){
      A_local.clear();
      iAfun_local.clear();
      jAvar_local.clear();
      td_constraint_list.get_constraint(i)->evaluate_sparse_A_matrix(timestep, opt_var_list, A_local, iAfun_local, jAvar_local);      

      constraint_index = td_constraint_list.get_constraint(i)->get_constraint_index();
      iAfun_absolute_start = timestep*num_time_dependent_constraints_funcs + constraint_index;     


      // Add to A_eval
      for(int j = 0; j < A_local.size(); j++){       
        iAfun_absolute_index = iAfun_absolute_start + iAfun_local[j];
        A_eval.push_back(A_local[j]);
        iAfun.push_back(iAfun_absolute_index);
        jAvar.push_back(jAvar_local[j]);       
      }

    }
  }


/* for(size_t i = 0; i < A.size(); i++){
    std::cout << "A(" << iAfun[i] << "," << jAvar[i] << ") = " << A[i] << std::endl;
 }*/

  // ---------------------------------------------------------------------------------------------
  // Evaluate Sparse Gradients for non-time dependent constraints


  // Number of non-negative A's
  neA = 0; // Presumably all values in A are zero.
}

/* 
int get_len_A(){
// Total non-zero elements of G.
}

int get_len_G(){
// Total length of G  
}

int get_len_neG(){
// Total non-zero elements of G.
}

*/

void WBDC_Opt::compute_G(std::vector<double> &G_eval, std::vector<int> &iGfun, std::vector<int> &jGvar, int &neG){
/*  std::vector<double> G_eval;
  std::vector<int> iGfun;
  std::vector<int> jGvar;*/

  std::vector<double> G_local;
  std::vector<int> iGfun_local;
  std::vector<int> jGvar_local;  

  int constraint_index = -1;
  int iGfun_absolute_start = -1;
  int iGfun_absolute_index = 0;
  // ---------------------------------------
  // Compute G of Time Dependent Constraints
  // ---------------------------------------
  int num_time_dependent_constraints_funcs = td_constraint_list.get_num_constraint_funcs(); // needs to be computed. For a single timestep, find the length of constraint functions, F
  //std::cout << "Total Number of Constraints: " << num_time_dependent_constraints_funcs << std::endl;   
  
  for(int timestep = 0; timestep < total_timesteps; timestep++){
    // Evaluate Known Constraint Gradient Elements
    for(int i = 0; i < td_constraint_list.get_size(); i++){
      G_local.clear();
      iGfun_local.clear();
      jGvar_local.clear();
      td_constraint_list.get_constraint(i)->evaluate_sparse_gradient(timestep, opt_var_list, G_local, iGfun_local, jGvar_local);      
      
      // Get G matrix, then identify the constraint's index number.
      // constraint_index is assigned by td_constraint_list during td_constraint_list.append. 
      constraint_index = td_constraint_list.get_constraint(i)->get_constraint_index();
      iGfun_absolute_start = timestep*num_time_dependent_constraints_funcs + constraint_index;

      //std::cout << " Timestep " << timestep << ", Constraint i: " << i << " has constraint index: " << iGfun_absolute_start  << std::endl; 
      // std::cout << "   absolute starting index: " << iGfun_absolute_start  << std::endl; 
   
      // Add to G_eval
      for(int j = 0; j < G_local.size(); j++){
        //std::cout << "G_local[j] = " << G_local[j] << std::endl;
        iGfun_absolute_index = iGfun_absolute_start + iGfun_local[j];
        G_eval.push_back(G_local[j]);
        iGfun.push_back(iGfun_absolute_index);
        jGvar.push_back(jGvar_local[j]);       
      }
    }
  }

/* for(size_t i = 0; i < G_eval.size(); i++){
    std::cout << "G(" << iGfun[i] << "," << jGvar[i] << ") = " << G_eval[i] << std::endl;
 }*/




  int iGfun_time_independent_funcs = total_timesteps*num_time_dependent_constraints_funcs;
  //std::cout << "Starting Index of Time Independent Constraint Functions:" << iGfun_time_independent_funcs << std::endl;
  // -------------------------------------------
  // Compute G of Non-Time Dependent Constraints
  // -------------------------------------------


  // -------------------------------------------
  // Compute G of Objective Constraint: Gradient calculation seems to be correct. 
  // -------------------------------------------

  #ifdef USE_OBJECTIVE_GRADIENT
    constraint_index = objective_function.objective_function_index;
    G_local.clear();
    iGfun_local.clear();
    jGvar_local.clear();  
    objective_function.evaluate_objective_gradient(opt_var_list, G_local, iGfun_local, jGvar_local);
    for(int j = 0; j < G_local.size(); j++){
      G_eval.push_back(G_local[j]);
      iGfun.push_back(constraint_index);
      jGvar.push_back(jGvar_local[j]);       
    }
  #endif



  // -------------------------------------------
  // Count number of non-zero elements in G.
  // -------------------------------------------  
  // int neG_counter = 0;
  // for (size_t i = 0; i < G_eval.size(); i++){
  //   if (G_eval[i] <= OPT_ZERO_GRADIENT_EPS){
  //     G_eval[i] = OPT_ZERO_GRADIENT_EPS;
  //   }else{
  //     neG_counter++;
  //   }
  // }
  //neG = neG_counter;
  neG = G_eval.size();

}








