#include "wbt_optimization.h"
#include "traj_solver.h"
#include <Utils/utilities.hpp>

WBT_Optimization* WBT_Optimization::GetWBT_Optimization(){
    static WBT_Optimization wbt_opt_obj;
    return &wbt_opt_obj;
}

WBT_Optimization::WBT_Optimization():m_q(NUM_Q), m_qdot(NUM_QDOT),
                                    m_tau(NUM_QDOT), cori_(NUM_QDOT),
                                    grav_(NUM_QDOT), A_(NUM_QDOT, NUM_QDOT), Ainv_(NUM_QDOT, NUM_QDOT){
  robot_model_ = RobotModel::GetRobotModel();	

  // Prepare Selection Matrix
  Sv.resize(NUM_VIRTUAL, NUM_QDOT);
  Sa.resize(NUM_ACT_JOINT, NUM_QDOT);
  Sv.setZero();
  Sa.setZero();

  Sv.block(0,0, NUM_VIRTUAL, NUM_VIRTUAL) = sejong::Matrix::Identity(NUM_VIRTUAL, NUM_VIRTUAL);
  Sa.block(0, NUM_VIRTUAL, NUM_ACT_JOINT, NUM_ACT_JOINT) = sejong::Matrix::Identity(NUM_ACT_JOINT, NUM_ACT_JOINT);

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

void WBT_Optimization::UpdateModel(const sejong::Vector &q, const sejong::Vector &qdot,
                                   sejong::Matrix &A_out, sejong::Vector &grav_out, sejong::Vector &cori_out){
  A_out.resize(NUM_QDOT, NUM_QDOT);
  grav_out.resize(NUM_QDOT, 1);
  cori_out.resize(NUM_QDOT, 1);
  A_out.setZero();
  grav_out.setZero();
  cori_out.setZero();  

  robot_model_->UpdateModel(q, qdot);
  robot_model_->getMassInertia(A_out); 
  robot_model_->getGravity(grav_out);  
  robot_model_->getCoriolis(cori_out); 
}



void WBT_Optimization::test_get_problem_functions(){
  std::cout << "Hello World!" << std::endl;
  std::cout << " Virtual x location: " << m_q[0] << std::endl;
}

void WBT_Optimization::run_solver_test(){
  snopt_solve_opt_problem();
}


void WBT_Optimization::prepare_state_problem_bounds(int &n, int &neF, int &ObjRow,
                                            std::vector<double> &xlow, std::vector<double> &xupp,
                                            std::vector<double> &Flow, std::vector<double> &Fupp){
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


  std::vector<double> xupp_states;
  std::vector<double> xlow_states; 
  // Set Number of States
  n = Fr_states.rows() + phi_states.rows();

  // Assign Reaction Force Bounds
  for(size_t i = 0; i < Fr_states.rows(); i++){
    Fr_upperbound[i] = 10000;
    Fr_lowerbound[i] = -10000;
    xupp_states.push_back(Fr_upperbound[i]);
    xlow_states.push_back(Fr_lowerbound[i]);    
  }

  // Assign Phi(q) bounds
  for(size_t i = 0; i < phi_states.rows(); i++){
    phi_upperbound[i] = 10;
    phi_lowerbound[i] = -1;    
    xupp_states.push_back(phi_upperbound[i]);
    xlow_states.push_back(phi_lowerbound[i]);        
  }

  // Assign to actual states
  xupp = xupp_states;  
  xlow = xlow_states;


  /* Problem Function order
  (1)              objective function
  (6)              WBC_virtual_states: Sv(Aqddot_des + b + g - U^t + Jc^Fr = 0)
  (17*2)           Fr contact constraint UFr >= 0
  (NUM_ACT_JOINT)  torque_constraints: -1800 <= Sa * (Aqddot_des + b + g - Jc^Fr) <= 1800
  (2)              Phi constraint phi*Fr = 0
  */
  sejong::Vector WBC_virtual_constraints(6); WBC_virtual_constraints.setZero();
  sejong::Vector Fr_contact_constraints(17*2); Fr_contact_constraints.setZero();
  sejong::Vector torque_constraints(NUM_ACT_JOINT); torque_constraints.setZero();
  sejong::Vector phi_constraints(2); phi_constraints.setZero();  

  std::vector<double> Fupp_;
  std::vector<double> Flow_; 
  neF = 1 + WBC_virtual_constraints.rows() + 
            Fr_contact_constraints.rows() + 
            torque_constraints.rows() + 
            phi_constraints.rows();

  //Assign Objective Function Bounds
  double inf = 1e20;
  Fupp_.push_back(inf);
  Flow_.push_back(-inf);  
  ObjRow = 0; // Assign Objective Row

  // Assign WBC Bounds
  for(size_t i = 0; i < WBC_virtual_constraints.rows(); i++){
    Fupp_.push_back(0);
    Flow_.push_back(0);    
  }

  // Assign Fr Contact constraint bounds
  for(size_t i = 0; i < Fr_contact_constraints.rows(); i++){
    Fupp_.push_back(inf);
    Flow_.push_back(0);    
  }

  // Assign Torque constraint bounds
  double torque_max = 1800.0;
  for(size_t i = 0; i < torque_constraints.rows(); i++){
    Fupp_.push_back(torque_max);
    Flow_.push_back(-torque_max);    
  }

  // Assign Phi Constraints
  for(size_t i = 0; i < phi_constraints.rows(); i++){
    Fupp_.push_back(0);
    Flow_.push_back(0);    
  }

  // Problem Function Bounds
  Fupp = Fupp_;
  Flow = Flow_;  

}

void WBT_Optimization::get_problem_functions(std::vector<double> &x, std::vector<double> &F, std::vector<double> &G){
  // Extract states x:
  sejong::Vector Fr_states(12); Fr_states.setZero(); // 6 Generalized Contact Forces
  sejong::Vector phi_states(2); Fr_states.setZero(); // 2 for each generalized contact

/*  int offset = 0;
  for(size_t i = 0; i < Fr_states.rows(); i++){
    Fr_states[i] = x[i + offset];
  }
  offset += Fr_states.rows();

  for(size_t i = 0; i < phi_states.rows(); i++){
    phi_states[i] = x[i + offset];
  }  */

  // Problem functions
  std::vector<double> F_;

  // Set Objective function: -----------------------------------------------------------------
  sejong::Matrix Q_Fr = sejong::Matrix::Identity(Fr_states.rows(), Fr_states.rows());
  Q_Fr(5,5) = 0.001; // Z direction 
  Q_Fr(11,11) = 0.001; // Z direction
  sejong::Vector objective_function = Fr_states.transpose()*Q_Fr*Fr_states;
  
  // Add to Problem Function
  F_.push_back(objective_function[0]);

  // Set WBC Virtual Constraints -------------------------------------------------------------
  sejong::Matrix A(NUM_QDOT, NUM_QDOT);
  sejong::Vector b(NUM_QDOT);
  sejong::Vector g(NUM_QDOT);
  UpdateModel(m_q, m_qdot, A, g, b);

  // Find desired actuation
  // needs tau_des =  A(B*xddot_des + c) + b + g
  sejong::Vector tau_des = b + g;
  sejong::Vector tau_act(NUM_QDOT); tau_act.setZero();
  tau_act.tail(NUM_ACT_JOINT) = tau_des.tail(NUM_ACT_JOINT);
  // Set Foot Contact Jacobian
  sejong::Matrix Jc(Fr_states.rows(), NUM_QDOT);
  sejong::Matrix Jtmp;
  robot_model_->getFullJacobian(m_q, SJLinkID::LK_rightCOP_Frame, Jtmp);
  Jc.block(0, 0, 6, NUM_QDOT) = Jtmp;
  robot_model_->getFullJacobian(m_q, SJLinkID::LK_leftCOP_Frame, Jtmp);
  Jc.block(6, 0, 6, NUM_QDOT) = Jtmp;  
  sejong::Vector WBC_virtual_constraints = Sv*(b + g - tau_act - Jc.transpose()*Fr_states);

  // Add to Problem Function
  for(size_t i = 0; i < WBC_virtual_constraints.size(); i++){
    F_.push_back(WBC_virtual_constraints[i]);
  }

  // Set UFr Contact Constraints -------------------------------------------------------------
  sejong::Matrix Uf;
  _UpdateUf(m_q, Uf);

  sejong::Vector UFr_constraints(17*2);
  UFr_constraints = Uf*Fr_states;

  sejong::pretty_print(UFr_constraints, std::cout, "UFr_constraints");

  // Add to Problem Function
  for(size_t i = 0; i < UFr_constraints.size(); i++){
    F_.push_back(UFr_constraints[i]);
  }

  // Set Torque Constraints -------------------------------------------------------------------
  sejong::Vector tau_constraints(NUM_ACT_JOINT);
  tau_constraints = tau_act.tail(NUM_ACT_JOINT);

  // Add to Problem Function
  for(size_t i = 0; i < tau_constraints.size(); i++){
    F_.push_back(tau_constraints[i]);
  }


}





bool WBT_Optimization::_UpdateUf(const sejong::Vector &q_state, sejong::Matrix &Uf_){
  double mu(0.8);
  double X(0.1);
  double Y(0.05);

  int size_u(17);
  double dim_contact_ = 12;
  Uf_ = sejong::Matrix::Zero(size_u*2, dim_contact_);

  sejong::Matrix U;
  _setU(X, Y, mu, U);
  Eigen::Quaternion<double> quat_tmp;

  robot_model_->getOrientation(q_state, SJLinkID::LK_rightCOP_Frame, quat_tmp);
  Eigen::Matrix3d R_rfoot_mtx(quat_tmp);
  sejong::Matrix R_rfoot(6,6); R_rfoot.setZero();
  R_rfoot.block(0,0, 3,3) = R_rfoot_mtx.transpose();
  R_rfoot.block(3,3, 3,3) = R_rfoot_mtx.transpose();

  robot_model_->getOrientation(q_state, SJLinkID::LK_leftCOP_Frame, quat_tmp);
  Eigen::Matrix3d R_lfoot_mtx(quat_tmp);
  sejong::Matrix R_lfoot(6,6); R_lfoot.setZero();
  R_lfoot.block(0,0, 3,3) = R_lfoot_mtx.transpose();
  R_lfoot.block(3,3, 3,3) = R_lfoot_mtx.transpose();

  Uf_.block(0,0, size_u, 6) = U * R_rfoot;
  Uf_.block(size_u, 6, size_u, 6) = U * R_lfoot;
  return true;
}

void WBT_Optimization::_setU(const double x, const double y, const double mu, sejong::Matrix & U){
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
  // ////////////////////////////////////////////////////

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