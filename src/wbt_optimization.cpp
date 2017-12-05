#include "wbt_optimization.h"
#include "traj_solver.h"

WBT_Optimization* WBT_Optimization::GetWBT_Optimization(){
    static WBT_Optimization wbt_opt_obj;
    return &wbt_opt_obj;
}

WBT_Optimization::WBT_Optimization():m_q(NUM_Q), m_qdot(NUM_QDOT),
                                    m_tau(NUM_QDOT), cori_(NUM_QDOT),
                                    grav_(NUM_QDOT), A_(NUM_QDOT, NUM_QDOT), Ainv_(NUM_QDOT, NUM_QDOT){
  robot_model_ = RobotModel::GetRobotModel();	

  m_q.setZero();  
  m_qdot.setZero();  
  m_tau.setZero();
  cori_.setZero(); 
  grav_.setZero(); 
  A_.setZero();
  Ainv_.setZero();  

  Initialization();

}

WBT_Optimization::~WBT_Optimization(){}

void WBT_Optimization::Initialization(){
  // Initialize to Valkyrie Standing Up

  // Set Virtual Joints
  // x_pos
  m_q[0] = 0.0;
  // y_pos
  m_q[1] = 0.0;
  // z_pos
  m_q[2] = 1.14; //1.135; //1.131; 
  m_q[NUM_Q - 1] = 1.0; // Pelvis Quaternion w = 1.0


  // Initialize Joints
  m_q[NUM_VIRTUAL + SJJointID::leftHipPitch] = -0.3; //r_joint_[r_joint_idx_map_.find("leftHipPitch"  )->second]->m_State.m_rValue[0] = -0.3;
  m_q[NUM_VIRTUAL + SJJointID::rightHipPitch] = -0.3;  //r_joint_[r_joint_idx_map_.find("rightHipPitch" )->second]->m_State.m_rValue[0] = -0.3;
  m_q[NUM_VIRTUAL + SJJointID::leftKneePitch] = 0.6;  //r_joint_[r_joint_idx_map_.find("leftKneePitch" )->second]->m_State.m_rValue[0] = 0.6;
  m_q[NUM_VIRTUAL + SJJointID::rightKneePitch] = 0.6;//r_joint_[r_joint_idx_map_.find("rightKneePitch")->second]->m_State.m_rValue[0] = 0.6;
  m_q[NUM_VIRTUAL + SJJointID::leftAnklePitch] = -0.3; //r_joint_[r_joint_idx_map_.find("leftAnklePitch")->second]->m_State.m_rValue[0] = -0.3;
  m_q[NUM_VIRTUAL + SJJointID::rightAnklePitch] = -0.3; //r_joint_[r_joint_idx_map_.find("rightAnklePitch")->second]->m_State.m_rValue[0] = -0.3;

  m_q[NUM_VIRTUAL + SJJointID::rightShoulderPitch] = 0.2; //r_joint_[r_joint_idx_map_.find("rightShoulderPitch")->second]->m_State.m_rValue[0] = 0.2;
  m_q[NUM_VIRTUAL + SJJointID::rightShoulderRoll] = 1.1;  //r_joint_[r_joint_idx_map_.find("rightShoulderRoll" )->second]->m_State.m_rValue[0] = 1.1;
  m_q[NUM_VIRTUAL + SJJointID::rightElbowPitch] = 0.4;  //r_joint_[r_joint_idx_map_.find("rightElbowPitch"   )->second]->m_State.m_rValue[0] = 0.4;
  m_q[NUM_VIRTUAL + SJJointID::rightForearmYaw] = 1.5;  //r_joint_[r_joint_idx_map_.find("rightForearmYaw" )->second]->m_State.m_rValue[0] = 1.5;

  m_q[NUM_VIRTUAL + SJJointID::leftShoulderPitch] = -0.2; //r_joint_[r_joint_idx_map_.find("rightShoulderPitch")->second]->m_State.m_rValue[0] = 0.2;
  m_q[NUM_VIRTUAL + SJJointID::leftShoulderRoll] = -1.1;  //r_joint_[r_joint_idx_map_.find("rightShoulderRoll" )->second]->m_State.m_rValue[0] = 1.1;
  m_q[NUM_VIRTUAL + SJJointID::leftElbowPitch] = -0.4;//0.4;  //r_joint_[r_joint_idx_map_.find("rightElbowPitch"   )->second]->m_State.m_rValue[0] = 0.4;
  m_q[NUM_VIRTUAL + SJJointID::leftForearmYaw] = 1.5;  //r_joint_[r_joint_idx_map_.find("rightForearmYaw" )->second]->m_State.m_rValue[0] = 1.5;	
  std::cout << "[WBT] Robot Starting State Initialized" << std::endl;
}


void WBT_Optimization::get_problem_functions(){
  std::cout << "Hello World!" << std::endl;
  std::cout << " Virtual x location: " << m_q[0] << std::endl;
}


void WBT_Optimization::run_solver_test(){
  snopt_solve_opt_problem();
}


void WBT_Optimization::prepare_state_problem_bounds(int &n, int &neF, double &ObjRow,
                                            double xlow[], double xupp[],
                                            double Flow[], double Fupp[]){
  /* State Order. All the sizes of the state vector are equal to the number of timesteps.
  std::vector<double> time_state;
  std::vector<sejong::Vector> Fr_states;
  std::vector<sejong::Vector> xddot_des_states;
  std::vector<sejong::Vector> Fn_states;
  std::vector<sejong::Vector> Fd_states;
  std::vector<sejong::Vector> phi_states;  
  std::vector<sejong::Vector> q_states;
  std::vector<sejong::Vector> qdot_states;
  */

  sejong::Vector Fr_states(12); Fr_states.setZero(); // 6 Generalized Contact Forces
  sejong::Vector phi_states(2); Fr_states.setZero(); // 2 for each generalized contact

  sejong::Vector Fr_upperbound(Fr_states.rows()); Fr_upperbound.setZero();
  sejong::Vector Fr_lowerbound(Fr_states.rows()); Fr_lowerbound.setZero(); 
  sejong::Vector phi_upperbound(phi_states.rows()); phi_upperbound.setZero();
  sejong::Vector phi_lowerbound(phi_states.rows()); phi_lowerbound.setZero();   

  // Set Number of States
  n = Fr_states.rows() + phi_states.rows();

  for(size_t i = 0; i < Fr_states.rows(); i++){
    Fr_upperbound[i] = 10000;
    Fr_lowerbound[i] = -10000;    
  }

  for(size_t i = 0; i < phi_states.rows(); i++){
    phi_upperbound[i] = 10;
    phi_lowerbound[i] = -1;    
  }


  /* Problem Function order
  (1)              objective function
  (6)              WBC_virtual_states: Sv(Aqddot_des + b + g - U^t + Jc^Fr = 0)
  (17*2)           Fr contact constraint UFr >= 0
  (NUM_ACT_JOINT) torque_constraints: -1800 <= Sa * (Aqddot_des + b + g - Jc^Fr) <= 1800
  */
  sejong::Vector WBC_virtual_constraints(6); WBC_virtual_constraints.setZero();
  sejong::Vector Fr_contact_constraints(17*2); Fr_contact_constraints.setZero();
  sejong::Vector torque_constraints(NUM_ACT_JOINT); torque_constraints.setZero();

  sejong::Vector WBC_virtual_constraints_upp(6); WBC_virtual_constraints_upp.setZero();
  sejong::Vector WBC_virtual_constraints_low(6); WBC_virtual_constraints_low.setZero();
  sejong::Vector Fr_contact_constraints_upp(17*2); Fr_contact_constraints_upp.setZero();
  sejong::Vector Fr_contact_constraints_low(17*2); Fr_contact_constraints_low.setZero();
  sejong::Vector torque_constraints_upp(NUM_ACT_JOINT); torque_constraints_upp.setZero();  
  sejong::Vector torque_constraints_low(NUM_ACT_JOINT); torque_constraints_low.setZero();    

}


/*
  Simple Problem: min Fr^T Q Fr
  Sv(Aqddot_des + b + g = U^t + Jc^Fr)
  UT = Aqddot_des + b + g

  tau = Sa * (Aqddot_des + b + g - Jc^Fr)
  UFr >= 0 
  Fr*Phi(q) = 0
 

  States: Fr1....Fr12, phi(q)
  Constraints rows(U) + rows(Sa) + Fr*phi


  Whole Body Dynamics:
    Aqddot + b + g = Utau + Jn*N*fn + Jd*D*fd

  LCP constraints on fn and fd

  Whole Body Controller  
    Aqddot_des + b + g = Jc^T Fr
    qddot_des = B xddot + c 

  states
  t
  Fn1, ..., Fn4
  Fd1, ..., Fd4
  q
  qdot

  Fr1...6, Fr7...12
  xddot1, ... xddot_dim_WBC



  t{n+1} = tn + dt;

}*/