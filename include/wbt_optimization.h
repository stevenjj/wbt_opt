#ifndef WBT_OPTIMIZATION_H
#define WBT_OPTIMIZATION_H

#include <stdio.h>
#include <string.h>
#include <iostream>

#include <Optimizer/snopt/include/snoptProblem.hpp>
#include "valkyrie_definition.h"
#include "RobotModel.hpp"

class WBT_Optimization{
public:
  static WBT_Optimization* GetWBT_Optimization();  
	RobotModel* robot_model_;

  void UpdateModel(const sejong::Vector &q, const sejong::Vector &qdot,
                                   sejong::Matrix &A_out, sejong::Vector &grav_out, sejong::Vector &cori_out);

	void Initialization();
  void test_get_problem_functions();


  void prepare_state_problem_bounds(int &n, int &neF, int &ObjRow,
                                            std::vector<double> &xlow, std::vector<double> &xupp,
                                            std::vector<double> &Flow, std::vector<double> &Fupp);
  void get_problem_functions(std::vector<double> &x, std::vector<double> &F, std::vector<double> &G);

  void run_solver_test();



  void simple_prepare_state_problem_bounds(int &n, int &neF, int &ObjRow,
                                            std::vector<double> &xlow, std::vector<double> &xupp,
                                            std::vector<double> &Flow, std::vector<double> &Fupp);
  void simple_get_problem_functions(std::vector<double> &x, std::vector<double> &F, std::vector<double> &G);


	sejong::Vector m_q;
  sejong::Vector m_qdot;
  sejong::Vector m_tau;

  sejong::Vector cori_;
  sejong::Vector grav_;
  sejong::Matrix A_;
  sejong::Matrix Ainv_;	



  sejong::Matrix Sv;
  sejong::Matrix Sa;

  int n_states_to_optimize;
  int neF_problems;  

  double dt;
  int    total_time_steps;


private:
  bool _UpdateUf(const sejong::Vector &q_state, sejong::Matrix &Uf_);
  void _setU(const double x, const double y, const double mu, sejong::Matrix & U);

	WBT_Optimization();
	~WBT_Optimization();	

};

#endif