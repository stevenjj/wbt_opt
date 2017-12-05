#include "wbt_optimization.h"
#include "traj_solver.h"

int main(int argc, char **argv)
{
	std::cout << "[WBT] Executed" << std::endl;
	WBT_Optimization* opt_obj_;
	opt_obj_ = WBT_Optimization::GetWBT_Optimization();

	// Change virtual x position, to test singleton 
	opt_obj_->m_q[0] = 100;

	opt_obj_->run_solver_test();	

//	snopt_solve_opt_problem();
	return 0;
}