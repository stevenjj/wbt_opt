#include <stdio.h>
#include <string.h>
#include <iostream>

#include <wbt/wbt_opt_variable.hpp>
#include <wbt/optimization_constants.hpp>

#include <wbt/containers/wbt_wholebody_task_list.hpp>
#include <wbt/tasks/wbt_task_com.hpp>
#include <wbt/tasks/wbt_task_leftfoot.hpp>
#include <wbt/tasks/wbt_task_rightfoot_righthand.hpp>


void test_wbt_opt_variable(){
	WBT_Opt_Variable xddot;	
	xddot.value = 10;
	std::cout << "[WBT] xddot test value:" << xddot.value << std::endl;
	std::cout << "[WBT] xddot name: " << xddot.name << std::endl;
	std::cout << "[WBT] xddot upper bound - lower u_bound: " << xddot.u_bound - xddot.l_bound  << std::endl;	
}

void test_wholebody_task_objects(){
	COM_Task com_task;
	LeftFoot_Task lf_task;	
	RightFoot_RightHand_Task rf_rh_task;		

	WholeBody_Task_List wholebody_task_list;

	wholebody_task_list.append_task(new COM_Task());
	wholebody_task_list.append_task(new LeftFoot_Task());
	wholebody_task_list.append_task(new RightFoot_RightHand_Task());		

	std::vector<Task*> wb_tasks;
	wholebody_task_list.get_task_list_copy(wb_tasks);

	std::cout << wb_tasks.size() << std::endl;
	for (size_t i = 0; i < wb_tasks.size(); i++){
		std::cout << "Task: " << i << " name: " << wb_tasks[i]->task_name << std::endl;	 
	}

}

int main(int argc, char **argv)
{
	std::cout << "[WBT] Hello world" << std::endl;

	test_wbt_opt_variable();
	test_wholebody_task_objects();

	return 0;
}