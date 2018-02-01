#include "wbt/snopt_wrapper_test.hpp"

int main(int argc, char **argv)
{
	std::cout << "[SNOPT Wrapper Test] Test Main" << std::endl;
	snopt_wrapper_test::solve_sample_problem();
	return 0;
}