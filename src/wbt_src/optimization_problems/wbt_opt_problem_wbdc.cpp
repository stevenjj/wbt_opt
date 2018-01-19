#include <wbt/optimization_problems/wbt_opt_problem_wbdc.hpp>
#include <Utils/utilities.hpp>

#include <wbt/tasks/wbt_task_leftfoot.hpp>
#include <wbt/contacts/wbt_contact_leftfoot.hpp>
#include <wbt/contacts/wbt_contact_rightfoot.hpp>

WBDC_Opt::WBDC_Opt(){
	robot_model = RobotModel::GetRobotModel();

	robot_q_init.resize(NUM_Q); 
	robot_qdot_init.resize(NUM_QDOT);	

	robot_q_init.setZero(); 
	robot_qdot_init.setZero();

	Initialization();
}

WBDC_Opt::~WBDC_Opt(){}

void WBDC_Opt::Initialization(){
	std::cout << "[WBDC_Opt] Initialization Called" << std::endl;
	initialize_starting_configuration();
	initialize_task_list();
	initialize_contact_list();
  initialize_constraint_list();  
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
	wb_task_list.append_task(new LeftFoot_Task());
  std::cout << "[WBDC_Opt] Task List Initialized" << std::endl;  
  wbc_constraint.set_task_list(&wb_task_list);

}

void WBDC_Opt::initialize_contact_list(){
  contact_list.append_contact(new LeftFoot_Contact());
  contact_list.append_contact(new RightFoot_Contact());
  std::cout << "[WBDC_Opt] Contact List Initialized" << std::endl;  
}

void WBDC_Opt::initialize_constraint_list(){
  std::cout << "[WBDC_Opt] test wbc constraint append" << std::endl;
  constraint_list.append_constraint(wbc_constraint);

  std::cout << "    Constraint Name:" << constraint_list.get_constraint(0)->constraint_name << std::endl; 

  std::cout << "       WBDC Task Size:" << std::endl; 
  constraint_list.get_constraint(0)->derived_test_function();

}

// Main class implementation
void WBDC_Opt::initialize_opt_vars(){
  // For each timestep:
    // timestep = 0 is a special case
    // Populate wbt_opt_variable_list to initialize x_value, xlow, xupp

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









