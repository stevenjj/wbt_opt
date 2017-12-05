#include "traj_solver.h"

void snopt_user_fun(int    *Status, int *n,    double x[],
       int    *needF,  int *neF,  double F[],
       int    *needG,  int *neG,  double G[],
       char      *cu,  int *lencu,
       int    iu[],    int *leniu,
       double ru[],    int *lenru) {

  WBT_Optimization* opt_obj;
  opt_obj = WBT_Optimization::GetWBT_Optimization();

  opt_obj->get_problem_functions();

  //==================================================================
  //
  //   Minimize     Fr^T I Fr
  //
  //   subject to   Fr_1 + Fr_2 <= 100
  //
  //==================================================================
  F[0] =  x[0]*x[0] + x[1]*x[1];
  F[1] =  x[0] + x[1];
}

void snopt_solve_opt_problem(){
  snoptProblemA traj_prob;	
  WBT_Optimization* opt_obj;
  opt_obj = WBT_Optimization::GetWBT_Optimization();

  double inf = 1.0e20;
  int Cold = 0, Basis = 1, Warm = 2;

  // Allocate and initialize;
  int n     =  2; // Number of Optimization Variables
  int neF   =  2; // Number of Problem Functions (constraints + objective function - state bounds constraints)

  int nS = 0, nInf;
  double sInf;

  double *x      = new double[n];
  double *xlow   = new double[n];
  double *xupp   = new double[n];
  double *xmul   = new double[n];
  int    *xstate = new    int[n];

  double *F      = new double[neF];
  double *Flow   = new double[neF];
  double *Fupp   = new double[neF];
  double *Fmul   = new double[neF];
  int    *Fstate = new int[neF];

  int    ObjRow  = 0; // The Objective Row
  double ObjAdd  = 0;  

  // opt_obj should fill in the upper and lower bounds
  // Upper and Lower Bounds of optimization variables
  xlow[0]   =  5;  xlow[1]   = 10;
  xupp[0]   = inf;  xupp[1]   =  inf;

  // Upper and lower bounds of problem functions
  Flow[0] = -inf; Flow[1] = 0;
  Fupp[0] =  inf; Fupp[1] = 100;

  traj_prob.initialize     ("", 1);  // no print file, summary on
  traj_prob.setProbName("Simple Fr Minimization");
  traj_prob.setPrintFile("traj.out"); 
  // snopta will compute the Jacobian by finite-differences.
  // snJac will be called  to define the
  // coordinate arrays (iAfun,jAvar,A) and (iGfun, jGvar).
  traj_prob.setIntParameter("Derivative option", 0);
  traj_prob.setIntParameter("Verify level ", 3);


  traj_prob.solve(Cold, neF, n, ObjAdd, ObjRow, snopt_user_fun,
			      xlow, xupp, Flow, Fupp,
    			  x, xstate, xmul, F, Fstate, Fmul,
    			  nS, nInf, sInf);

  std::cout << "Found Solutions:" << x[0] << " " << x[1] << std::endl; 

}
