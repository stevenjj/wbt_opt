#include "traj_solver.h"

void test_get_funcs(){
  WBT_Optimization* opt_obj;
  opt_obj = WBT_Optimization::GetWBT_Optimization();  

  std::cout << "(n_states, neF) = (" << opt_obj->n_states_to_optimize << ", " <<
                                  opt_obj->neF_problems << ")" << std::endl;

  std::vector<double> x_in;
  std::vector<double> F_out;
  std::vector<double> G_out;

  int n_states = opt_obj->n_states_to_optimize;
  int neF_probs = opt_obj->neF_problems;

  for(size_t i = 0; i < n_states; i++){
    x_in.push_back(0);
  }

  opt_obj->get_problem_functions(x_in, F_out, G_out);

  std::cout << "F_out size:" << F_out.size() << std::endl;
  double *F      = new double[neF_probs];
  for(size_t i = 0; i < neF_probs; i++){
    F[i] = F_out[i];
  }



}


void snopt_user_fun(int    *Status, int *n,    double x[],
       int    *needF,  int *neF,  double F[],
       int    *needG,  int *neG,  double G[],
       char      *cu,  int *lencu,
       int    iu[],    int *leniu,
       double ru[],    int *lenru) {

  WBT_Optimization* opt_obj;
  opt_obj = WBT_Optimization::GetWBT_Optimization();

  opt_obj->test_get_problem_functions();
  test_get_funcs();

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

void test_bounds_prep(){
  WBT_Optimization* opt_obj;
  opt_obj = WBT_Optimization::GetWBT_Optimization();  
  std::vector<double> xlow_vec;
  std::vector<double> xupp_vec;
  std::vector<double> Flow_vec; 
  std::vector<double> Fupp_vec;

  // Allocate and initialize;
  int n     =  0; // Number of Optimization Variables
  int neF   =  0; // Number of Problem Functions (constraints + objective function - state bounds constraints)
  int ObjRow = 0;
  int nS = 0, nInf;
  double sInf;

  opt_obj->prepare_state_problem_bounds(n, neF, ObjRow, xlow_vec, xupp_vec, Flow_vec, Fupp_vec);

  double *x      = new double[n];
  double *xlow   = new double[n];
  double *xupp   = new double[n];
  double *Flow   = new double[neF];
  double *Fupp   = new double[neF];

  for(size_t i = 0; i < n; i++){
    xlow[i] = xlow_vec[i];
    xupp[i] = xupp_vec[i];
  }

  for(size_t i = 0; i < neF; i++){
    Flow[i] = Flow_vec[i];
    Fupp[i] = Fupp_vec[i];              
  }

}

void test_snopt_wbdc_fun(int    *Status, int *n,    double x[],
       int    *needF,  int *neF,  double F[],
       int    *needG,  int *neG,  double G[],
       char      *cu,  int *lencu,
       int    iu[],    int *leniu,
       double ru[],    int *lenru){

  WBT_Optimization* opt_obj;
  opt_obj = WBT_Optimization::GetWBT_Optimization();  

  std::vector<double> x_in;
  std::vector<double> F_out;
  std::vector<double> G_out;

  int n_states = opt_obj->n_states_to_optimize;
  int neF_probs = opt_obj->neF_problems;

  // Populate x states
  for(size_t i = 0; i < n_states; i++){
    x_in.push_back(x[i]);
  }

  opt_obj->get_problem_functions(x_in, F_out, G_out);
//  opt_obj->simple_get_problem_functions(x_in, F_out, G_out);

  for(size_t i = 0; i < neF_probs; i++){
    F[i] = F_out[i];
//    std::cout << "F[i] = " << F_out[i] << std::endl;
  }


}

void test_snopt_solve_wbdc(){
  snoptProblemA wbdc_prob;   
  WBT_Optimization* opt_obj;
  opt_obj = WBT_Optimization::GetWBT_Optimization();  
  std::vector<double> xlow_vec;
  std::vector<double> xupp_vec;
  std::vector<double> Flow_vec; 
  std::vector<double> Fupp_vec;
  std::vector<double> x_state_guess;

  // Allocate and initialize;
  int Cold = 0, Basis = 1, Warm = 2;
  int n     =  0; // Number of Optimization Variables
  int neF   =  0; // Number of Problem Functions (constraints + objective function - state bounds constraints)
  int nS = 0, nInf;
  double sInf;
  int ObjRow = 0;
  double ObjAdd  = 0; 

  opt_obj->prepare_state_problem_bounds(n, neF, ObjRow, xlow_vec, xupp_vec, Flow_vec, Fupp_vec);
  opt_obj->initialize_state_guess(x_state_guess);
  //opt_obj->simple_prepare_state_problem_bounds(n, neF, ObjRow, xlow_vec, xupp_vec, Flow_vec, Fupp_vec);  

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

  // Do some initializations on guess
  //x[69+6-1] = 680.0;
  //x[69+12-1] = 680.0;
  for(size_t i = 0; i < n; i++){
    x[i] = x_state_guess[i];
    std::cout << "x_state_guess[" << i << "]: " << x_state_guess[i] << std::endl;
  }

  for(size_t i = 0; i < n; i++){
    xlow[i] = xlow_vec[i];
    xupp[i] = xupp_vec[i];
  }

  for(size_t i = 0; i < neF; i++){
    Flow[i] = Flow_vec[i];
    Fupp[i] = Fupp_vec[i];              
  }

  for(size_t i = 0; i < neF; i++){
    std::cout << "i:" << i << " F minimums:" << Flow[i] << std::endl;
  }

  for(size_t i = 0; i < neF; i++){
    std::cout << "i:" << i << " F maximums:" << Fupp[i] << std::endl;
  }

  wbdc_prob.initialize     ("", 1);  // no print file, summary on
  wbdc_prob.setProbName("WBDC Fr Minimization");
  wbdc_prob.setPrintFile("WBDC.out"); 
  // snopta will compute the Jacobian by finite-differences.
  // snJac will be called  to define the
  // coordinate arrays (iAfun,jAvar,A) and (iGfun, jGvar).
  wbdc_prob.setIntParameter("Derivative option", 0);
  wbdc_prob.setIntParameter("Verify level ", 3);



  int *iAfun_test;
  int *jAvar_test;
  double *A_test;
  int    neA_test;

  int *iGfun_test;
  int *jGvar_test;
  double *G_test;
  int    neG_test;

  wbdc_prob.computeJac(neF, n, test_snopt_wbdc_fun, x, xlow, xupp,
      iAfun_test, jAvar_test, A_test, neA_test,
      iGfun_test, jGvar_test, neG_test);
  std::cout << "Size of non zero A's: " << neA_test << std::endl;
  std::cout << "Size of non zero G's: " << neG_test << std::endl; 



  wbdc_prob.solve(Cold, neF, n, ObjAdd, ObjRow, test_snopt_wbdc_fun,
            xlow, xupp, Flow, Fupp,
            x, xstate, xmul, F, Fstate, Fmul,
            nS, nInf, sInf);  

  std::cout << "Found Solutions:" << std::endl;
  for(size_t i = 0; i < n; i++){
   std::cout << x[i] << std::endl; 
  }

}

void snopt_solve_opt_problem(){
  snoptProblemA traj_prob;	
  WBT_Optimization* opt_obj;
  opt_obj = WBT_Optimization::GetWBT_Optimization();

  test_bounds_prep();

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
