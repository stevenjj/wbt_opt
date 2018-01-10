#include <stdio.h>
#include <string.h>
#include <iostream>

#include <wbt/wbt_opt_variable.hpp>
#include <wbt/optimization_constants.hpp>


#include <wbt/tasks/wbt_task_com.hpp>
#include <wbt/tasks/wbt_task_leftfoot.hpp>
#include <wbt/tasks/wbt_task_rightfoot_righthand.hpp>

int main(int argc, char **argv)
{
	std::cout << "[WBT] Hello world" << std::endl;
	WBT_Opt_Variable xddot;

	COM_Task com_task;
	LeftFoot_Task lf_task;	
	RightFoot_RightHand_Task rf_rh_task;		

	xddot.value = 10;
	std::cout << "[WBT] xddot test value:" << xddot.value << std::endl;
	std::cout << "[WBT] xddot name: " << xddot.name << std::endl;
	std::cout << "[WBT] xddot upper bound - lower u_bound: " << xddot.u_bound - xddot.l_bound  << std::endl;	

	return 0;
}