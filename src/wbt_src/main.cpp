#include <stdio.h>
#include <string.h>
#include <iostream>

#include <wbt/optimization_constants.hpp>

#include <wbt/containers/wbt_opt_variable.hpp>
#include <wbt/containers/wbt_wholebody_task_list.hpp>
#include <wbt/tasks/wbt_task_com.hpp>
#include <wbt/tasks/wbt_task_leftfoot.hpp>
#include <wbt/tasks/wbt_task_rightfoot_righthand.hpp>
#include <wbt/soft_constraints/wbt_keyframe.hpp>
#include <wbt/soft_constraints/wbt_keyframe_position.hpp>
#include <wbt/soft_constraints/wbt_keyframe_orientation.hpp>


void test_wbt_opt_variable(){
	WBT_Opt_Variable xddot;	
	xddot.value = 10;
	std::cout << "[WBT] xddot test value:" << xddot.value << std::endl;
	std::cout << "[WBT] xddot name: " << xddot.name << std::endl;
	std::cout << "[WBT] xddot upper bound - lower u_bound: " << xddot.u_bound - xddot.l_bound  << std::endl;	
}

void test_wholebody_task_objects(){
	std::cout << "[WBT] Testing task initialization" << std::endl;
	COM_Task com_task;
	LeftFoot_Task lf_task;	
	RightFoot_RightHand_Task rf_rh_task;		
	std::cout << "Successful" << std::endl;

	std::cout << "[WBT] Testing wholebody task container" << std::endl;
	WholeBody_Task_List wholebody_task_list;
	wholebody_task_list.append_task(new COM_Task());
	wholebody_task_list.append_task(new LeftFoot_Task());
	wholebody_task_list.append_task(new RightFoot_RightHand_Task());		


	std::cout << "[WBT] Testing wbt container copy" << std::endl;
	std::vector<Task*> wb_tasks;
	wholebody_task_list.get_task_list_copy(wb_tasks);
	std::cout << "  Copy size: " << wb_tasks.size() << " should be 3" << std::endl;
	for (size_t i = 0; i < wb_tasks.size(); i++){
		std::cout << "  Task: " << i << " name: " << wb_tasks[i]->task_name << std::endl;	 
	}

	std::cout << "[WBT] Testing wholebody task container retrieval" << std::endl;
	for (size_t i = 0; i < wholebody_task_list.get_size(); i++){
		std::cout << "  Task: " << i << " name: " << wholebody_task_list.get_task(i)->task_name << std::endl;	 
	}

}

void test_wbt_keyframe(){
	std::cout << "[WBT] Testing KeyFrame object" << std::endl;
	KeyFrame sample_kf;
	std::string keyframe_name;

	sample_kf.get_keyframe_name(keyframe_name);
	std::cout << "Keyframe name: " << keyframe_name << std::endl;

	Position_KeyFrame pos_kf;
	std::cout << "Position KF timestep:" << pos_kf.timestep << std::endl;

	sejong::Vect3 desired_keyframe;
	Position_KeyFrame sample_pos_kf("HandPos", 4, desired_keyframe);
	std::cout << "Position Hand KF name:" << sample_pos_kf.keyframe_name << std::endl;
	std::cout << "Position Hand KF link id:" << sample_pos_kf.get_link_id() << std::endl;	

	sejong::Quaternion desired_ori_keyframe;
	Orientation_KeyFrame sample_ori_kf("HandOri", 3, desired_ori_keyframe);
	std::cout << "Orientation Hand KF name:" << sample_ori_kf.keyframe_name << std::endl;
	std::cout << "Orientation Hand KF link id:" << sample_ori_kf.get_link_id() << std::endl;	


}

int main(int argc, char **argv)
{
	std::cout << "[WBT] Testing object and argument calls" << std::endl;

	test_wbt_opt_variable();
	test_wholebody_task_objects();
	test_wbt_keyframe();


	return 0;
}