#include <wbt/wbt_snopt_wrapper.hpp>
#include <wbt/optimization_problems/wbt_opt_problem_wbdc.hpp>

int main(int argc, char **argv)
{
	std::cout << "[Main] Running Optimization Problem" << std::endl;
	Optimization_Problem_Main* 	wbc_problem = new WBDC_Opt();

	snopt_wrapper::solve_problem(wbc_problem);
	//snopt_wrapper::solve_problem_no_gradients(wbc_problem);

	delete wbc_problem;
	return 0;
}