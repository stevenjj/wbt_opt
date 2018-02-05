#include <wbt/wbt_snopt_wrapper.hpp>

namespace snopt_wrapper{

  Optimization_Problem_Main* ptr_optimization_problem;

  void wbt_FG(int    *Status, int *n,    double x[],
     int    *needF,  int *lenF,  double F[],
     int    *needG,  int *lenG,  double G[],
     char      *cu,  int *lencu,
     int    iu[],    int *leniu,
     double ru[],    int *lenru){

		std::vector<double> x_vars;
		std::vector<double> F_eval;
		std::vector<double> G_eval;				
		std::vector<int> iGfun_eval;
		std::vector<int> jGvar_eval;	
		int neG_eval = 0;							

	  	// Update optimization x_vars
		for (size_t i = 0; i < *n; i++){
			x_vars.push_back(x[i]);
		}
		ptr_optimization_problem->update_opt_vars(x_vars);
		// Get F and G evaluations
		if ((*needF) > 0){
			ptr_optimization_problem->compute_F(F_eval);
		}
		if ((*needG) > 0){
			ptr_optimization_problem->compute_G(G_eval, iGfun_eval, jGvar_eval, neG_eval);
		}


	// Populate F
	for (size_t i = 0; i < F_eval.size(); i++){
		F[i] = F_eval[i];
	}
	//Populate G
	for (size_t i = 0; i < G_eval.size(); i++){
		G[i] = G_eval[i];
	}	



    }

  void solve_problem(Optimization_Problem_Main* input_ptr_optimization_problem){
  	std::cout << "[SNOPT Wrapper] Initializing Optimization Problem" << std::endl;
	ptr_optimization_problem = input_ptr_optimization_problem;
  	std::cout << "[SNOPT Wrapper] Problem Name: " << ptr_optimization_problem->problem_name << std::endl;


	// Prepare Variable Containers
	std::vector<double> x_vars;
	std::vector<double> x_vars_low;
	std::vector<double> x_vars_upp;	

	std::vector<double> F_eval;
	std::vector<double> F_eval_low;
	std::vector<double> F_eval_upp;

	std::vector<double> G_eval;
	std::vector<int> iGfun_eval;
	std::vector<int> jGvar_eval;
	int neG_eval = 0;

	std::vector<double> A_eval;
	std::vector<int> iAfun_eval;
	std::vector<int> jAvar_eval;
	int neA_eval = 0;


	ptr_optimization_problem->get_init_opt_vars(x_vars);
	std::cout << "[SNOPT Wrapper] Initialized Initial Value of Optimization Variables" << std::endl;
	std::cout << "[SNOPT Wrapper]                    Number of Optimization Variables: " << x_vars.size() << std::endl;	

	ptr_optimization_problem->get_opt_vars_bounds(x_vars_low, x_vars_upp);
	std::cout << "[SNOPT Wrapper] Initialized Bounds of Optimization Variables" << std::endl;
	std::cout << "[SNOPT Wrapper]  						  Num of Lower Bounds: " << x_vars_low.size() << std::endl;	
	std::cout << "[SNOPT Wrapper]  						  Num of Upper Bounds: " << x_vars_upp.size() << std::endl;		

	ptr_optimization_problem->get_F_bounds(F_eval_low, F_eval_upp);
	std::cout << "[SNOPT Wrapper] Initialized Bounds of Functions" << std::endl;
	std::cout << "[SNOPT Wrapper]  			  Num of Lower Bounds: " << F_eval_low.size() << std::endl;	
	std::cout << "[SNOPT Wrapper]  			  Num of Upper Bounds: " << F_eval_upp.size() << std::endl;		

	if (F_eval_low.size() == F_eval_upp.size()){
		std::cout << "[SNOPT_Wrapper] There are " << F_eval_low.size() << " problem functions" << std::endl; 
	}else{
		std::cout << "[SNOPT Wrapper] Error! Bounds are not equal" << std::endl;
		throw;
	}


	// Compute F initially
	ptr_optimization_problem->compute_F(F_eval);


	//Compute G and A to initialize gradient variables
	std::cout << "[SNOPT Wrapper] Computing Known Gradients" << std::endl;
	ptr_optimization_problem->compute_G(G_eval, iGfun_eval, jGvar_eval, neG_eval);
	std::cout << "[SNOPT Wrapper] Finished. Number of Known Grad G Elements: " << G_eval.size() <<  std::endl;
	std::cout << "[SNOPT Wrapper] Size of G indices (iGfun, jGvar) " << "(" << iGfun_eval.size() << ", " << jGvar_eval.size() << ")" <<  std::endl;	
	std::cout << "[SNOPT Wrapper] Number of non-zero elements: " << neG_eval <<  std::endl;

	std::cout << "[SNOPT Wrapper] Computing Linear Gradients" << std::endl;
	ptr_optimization_problem->compute_A(A_eval, iAfun_eval, jAvar_eval, neA_eval);
	std::cout << "[SNOPT Wrapper] Finished. Number of Known Grad A Elements: " << A_eval.size() <<  std::endl;
	std::cout << "[SNOPT Wrapper] Size of A indices (iAfun, jAvar) " << "(" << iAfun_eval.size() << ", " << jAvar_eval.size() << ")" <<  std::endl;	
	std::cout << "[SNOPT Wrapper] Number of non-zero elements: " << neA_eval <<  std::endl;

/*
	// Test update and populate x_vars
	// Update x_vars
	std::vector<double> x_updated_vars;
	x_updated_vars = x_vars;
	ptr_optimization_problem->update_opt_vars(x_updated_vars);

	// Populate x_vars
	std::vector<double> x_populated_vars;
	ptr_optimization_problem->get_current_opt_vars(x_populated_vars);
	std::cout << "[SNOPT Wrapper] Size of populated x_vars: " << x_populated_vars.size() << std::endl; 

*/
	int Cold  = 0;
	
	// Get Sizes
	// int n = 2;
	// int nF = 2;
	int n = x_vars.size(); // Size of optimization variables
	int nF = F_eval_low.size(); // Size of Constraints

	int lenG = G_eval.size();
	int lenA = A_eval.size();	
	int neA = neA_eval;
	int neG = neG_eval;	

	std::cout << "lenG, lenA = " <<  lenG << "," << lenA << std::endl;


	// Initialize Variables
	double *x      = new double[n];
	double *xlow   = new double[n];
	double *xupp   = new double[n];
	double *xmul   = new double[n];
	int    *xstate = new    int[n];

	double *F      = new double[nF];
	double *Flow   = new double[nF];
	double *Fupp   = new double[nF];
	double *Fmul   = new double[nF];
	int    *Fstate = new int[nF];

	int *iAfun = new int[lenA];
	int *jAvar = new int[lenA];
	double *A  = new double[lenA];

	int *iGfun = new int[lenG];
	int *jGvar = new int[lenG];

	// Get Objective Row
	int ObjRow = 0;
	ptr_optimization_problem->get_F_obj_Row(ObjRow);
	double ObjAdd = 0.0;

	int nS = 0, nInf = 0;

	int jx1, jx2, ju, ode1, ode2, Obj;
	double sInf;
	double inf = OPT_INFINITY;	


	// Populate x_vars
	for(size_t i = 0; i < x_vars.size(); i++){
		x[i] = x_vars[i];		
		xstate[i] = x[i];
		xmul[i] = 0.0;
		std::cout << "x[" << i << "] = " << x[i] << std::endl;
	}
	// Populate F bounds
	for (size_t i = 0; i < F_eval.size(); i++){
		F[i] = F_eval[i];
		Fstate[i] =  F_eval[i];
		Fmul[i] = 0.0;
	}
	// Populate x bounds
	for(size_t i = 0; i < x_vars_low.size(); i++){
		xlow[i] = x_vars_low[i];		
	}
	for(size_t i = 0; i < x_vars_upp.size(); i++){
		xupp[i] = x_vars_upp[i];		
	}
	// Populate F bounds
	for (size_t i = 0; i < F_eval_low.size(); i++){
		Flow[i] = F_eval_low[i];
	}
	for (size_t i = 0; i < F_eval_upp.size(); i++){
		Fupp[i] = F_eval_upp[i];
	}	
	// Populate G indices
	for(size_t i = 0; i < iGfun_eval.size(); i++){
		iGfun[i] =  iGfun_eval[i];
	}
	for(size_t i = 0; i < jGvar_eval.size(); i++){
		jGvar[i] =  jGvar_eval[i];
	}	
	// Populate A and its indices
	for(size_t i = 0; i < A_eval.size(); i++){
		A[i] = A_eval[i];
	}
	for(size_t i = 0; i < iAfun_eval.size(); i++){
		iAfun[i] = iAfun_eval[i];
	}	
	for(size_t i = 0; i < jAvar_eval.size(); i++){
		jAvar[i] = jAvar_eval[i];
	}	


	  // xlow[0]   =  5;  xlow[1]   = 10;
	  // xupp[0]   = inf;  xupp[1]   =  inf;

	  // // Upper and lower bounds of problem functions
	  // Flow[0] = -inf; Flow[1] = 0;
	  // Fupp[0] =  inf; Fupp[1] = 100;


	int *iAfun_test;
	int *jAvar_test;
	double *A_test;
	int    neA_test;

	int *iGfun_test;
	int *jGvar_test;
	double *G_test;
	int    neG_test;



	snoptProblemA whole_body_trajectory_problem;
	whole_body_trajectory_problem.initialize     ("", 1);  // no print file, summary on
  	whole_body_trajectory_problem.setProbName(ptr_optimization_problem->problem_name.c_str());
  	whole_body_trajectory_problem.setPrintFile("wbt_problem.out"); 
	whole_body_trajectory_problem.setIntParameter("Derivative option", 0);
	whole_body_trajectory_problem.setIntParameter("Verify level ", 3);	

	whole_body_trajectory_problem.solve(Cold, nF, n, ObjAdd, ObjRow, snopt_wrapper::wbt_FG,
			  iAfun, jAvar, A, neA,
			  iGfun, jGvar, neG,
			  xlow, xupp, Flow, Fupp,
			  x, xstate, xmul,
			  F, Fstate, Fmul,
			  nS, nInf, sInf);


	// whole_body_trajectory_problem.computeJac(nF, n, snopt_wrapper::wbt_FG, x, xlow, xupp,
	// 	  iAfun_test, jAvar_test, A_test, neA_test,
	// 	  iGfun_test, jGvar_test, neG_test);

	// std::cout << "Size of non zero A's: " << neA_test << std::endl;
	// std::cout << "Size of non zero G's: " << neG_test << std::endl;	



  	// whole_body_trajectory_problem.solve(Cold, nF, n, ObjAdd, ObjRow, snopt_wrapper::wbt_FG,
			//       xlow, xupp, Flow, Fupp,
   //  			  x, xstate, xmul, F, Fstate, Fmul,
   //  			  nS, nInf, sInf);


	// for (size_t i = 0; i < n; i++){
	// 	std::cout << "x[" << i << "] = " << x[i] << std::endl;
	// }


	// for (size_t i = 0; i < nF; i++){
	// 	std::cout << "F[" << i << "] = " << F[i] << std::endl;
	// }	



	delete []iAfun;  delete []jAvar;  delete []A;
	delete []iGfun;  delete []jGvar;

	delete []x;      delete []xlow;   delete []xupp;
	delete []xmul;   delete []xstate;

	delete []F;      delete []Flow;   delete []Fupp;
	delete []Fmul;   delete []Fstate;


  }

}