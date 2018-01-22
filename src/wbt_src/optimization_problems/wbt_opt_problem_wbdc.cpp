#include <wbt/optimization_problems/wbt_opt_problem_wbdc.hpp>
#include <Utils/utilities.hpp>

#include <wbt/tasks/wbt_task_leftfoot.hpp>
#include <wbt/tasks/wbt_task_rightfoot.hpp>
#include <wbt/contacts/wbt_contact_leftfoot.hpp>
#include <wbt/contacts/wbt_contact_rightfoot.hpp>

WBDC_Opt::WBDC_Opt(){
	robot_model = RobotModel::GetRobotModel();

	robot_q_init.resize(NUM_Q); 
	robot_qdot_init.resize(NUM_QDOT);	

	robot_q_init.setZero(); 
	robot_qdot_init.setZero();

  ptr_wbc_constraint = new Wholebody_Controller_Constraint();
  Initialization();
}

WBDC_Opt::~WBDC_Opt(){
}

void WBDC_Opt::Initialization(){
	std::cout << "[WBDC_Opt] Initialization Called" << std::endl;
	total_timesteps = 1;
  initialize_starting_configuration();
	initialize_task_list();
	initialize_contact_list();
  initialize_constraint_list();  


  initialize_opt_vars();
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

void WBDC_Opt::initialize_constraint_list(){
  std::cout << "[WBDC_Opt] Initializing WBC Constraints" << std::endl;

  ptr_wbc_constraint->set_task_list(&wb_task_list);
  ptr_wbc_constraint->set_contact_list(&contact_list);  
//  constraint_list.append_constraint(new Wholebody_Controller_Constraint(&wb_task_list, &contact_list));
  constraint_list.append_constraint(ptr_wbc_constraint);  


  // Test WBC B and c matrix construction
/*  sejong::Matrix B_test;
  sejong::Vector c_test;  
  constraint_list.get_constraint(0)->test_function2(robot_q_init, robot_qdot_init, B_test, c_test);*/

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
        WBT_Opt_Variable* xddot_var = new WBT_Opt_Variable("xddot", VAR_TYPE_TA, i, 0.0, -OPT_INFINITY, OPT_INFINITY);
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

    // Key Frame List Initialization
    // Insert initialization here
    //

  }


  std::cout << "[WBDC_OPT] Total number of optimization variables: " << opt_var_list.get_size() << std::endl;


}


void WBDC_Opt::initialize_F_bounds(){
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

void WBDC_Opt::compute_F_objective_function(){
  // compute_objective_function(&wbt_opt_var_list)
}

void WBDC_Opt::compute_F_constraints(){
  // We know the size of F.
  // Compute F(timestep, wbt_opt_var_list)

}









