#include <wbt/wbt_snopt_wrapper.hpp>

namespace snopt_wrapper{
  Optimization_Problem_Main* ptr_optimization_problem;

  void wbt_FG(int    *Status, int *n,    double x[],
     int    *needF,  int *lenF,  double F[],
     int    *needG,  int *lenG,  double G[],
     char      *cu,  int *lencu,
     int    iu[],    int *leniu,
     double ru[],    int *lenru){

     }

  void solve_problem(Optimization_Problem_Main* input_ptr_optimization_problem){
  	std::cout << "[SNOPT Wrapper] Initializing Optimization Problem" << std::endl;
	ptr_optimization_problem = input_ptr_optimization_problem;
  	std::cout << "[SNOPT Wrapper] Problem Name: " << ptr_optimization_problem->problem_name << std::endl;


/*	snoptProblemA catmixa;

	int Cold  = 0;

	int nH    = 1000;     // Do not need
	int n     = 3*(nH+1); // Number of variables: size of var_list
	int nCon  = 2*nH;	  // Do not need
	int nF    = nCon + 1; // total number of functions

	int    ObjRow; 		  // get objective row
	double ObjAdd = -1.0;

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

	int lenA   = 2;
	int *iAfun = new int[lenA];
	int *jAvar = new int[lenA];
	double *A  = new double[lenA];

	int lenG   = 14*nH + 1;
	int *iGfun = new int[lenG];
	int *jGvar = new int[lenG];

	int nS = 0, nInf = 0, neA = 0, neG = 0;
	int jx1, jx2, ju, ode1, ode2, Obj;
	double sInf;

	double inf = 1.0e20;*/

	// Prepare Variable Containers
	std::vector<double> x_vars;
	std::vector<double> x_low;
	std::vector<double> x_upp;	

	std::vector<double> F_eval;
	std::vector<double> F_low;
	std::vector<double> F_upp;

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

	ptr_optimization_problem->get_opt_vars_bounds(x_low, x_upp);
	std::cout << "[SNOPT Wrapper] Initialized Bounds of Optimization Variables" << std::endl;
	std::cout << "[SNOPT Wrapper]  						  Num of Lower Bounds: " << x_low.size() << std::endl;	
	std::cout << "[SNOPT Wrapper]  						  Num of Upper Bounds: " << x_upp.size() << std::endl;		

	ptr_optimization_problem->get_F_bounds(F_low, F_upp);
	std::cout << "[SNOPT Wrapper] Initialized Bounds of Functions" << std::endl;
	std::cout << "[SNOPT Wrapper]  			  Num of Lower Bounds: " << F_low.size() << std::endl;	
	std::cout << "[SNOPT Wrapper]  			  Num of Upper Bounds: " << F_upp.size() << std::endl;		

	if (F_low.size() == F_upp.size()){
		std::cout << "[SNOPT_Wrapper] There are " << F_low.size() << " problem functions" << std::endl; 
	}

	// Compute G and A to initialize gradient variables
	std::cout << "[SNOPT Wrapper] Computing Known Gradients" << std::endl;
	ptr_optimization_problem->compute_G(G_eval, iGfun_eval, jGvar_eval, neG_eval);
	std::cout << "[SNOPT Wrapper] Finished. Number of Known Grad G Elements: " << G_eval.size() <<  std::endl;
	std::cout << "[SNOPT Wrapper] Number of non-zero elements: " << neG_eval <<  std::endl;

	std::cout << "[SNOPT Wrapper] Computing Linear Gradients" << std::endl;
	ptr_optimization_problem->compute_A(A_eval, iAfun_eval, jAvar_eval, neA_eval);
	std::cout << "[SNOPT Wrapper] Finished. Number of Known Grad A Elements: " << A_eval.size() <<  std::endl;
	std::cout << "[SNOPT Wrapper] Number of non-zero elements: " << neA_eval <<  std::endl;

	// Populate x_vars





  }

}
