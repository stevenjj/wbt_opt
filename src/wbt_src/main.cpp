#include <stdio.h>
#include <string.h>
#include <iostream>

#include <wbt/optimization_constants.hpp>

#include <wbt/containers/wbt_opt_variable.hpp>
#include <wbt/containers/wbt_wholebody_task_list.hpp>
#include <wbt/containers/wbt_keyframe_list.hpp>
#include <wbt/containers/wbt_contact_list.hpp>

#include <wbt/tasks/wbt_task_com.hpp>
#include <wbt/tasks/wbt_task_leftfoot.hpp>
#include <wbt/tasks/wbt_task_rightfoot_righthand.hpp>
#include <wbt/soft_constraints/wbt_keyframe.hpp>
#include <wbt/soft_constraints/wbt_keyframe_position.hpp>
#include <wbt/soft_constraints/wbt_keyframe_orientation.hpp>

#include <wbt/hard_constraints/wbt_contact_wrench_constraint.hpp>
#include <wbt/hard_constraints/wbt_contact_lcp_constraint.hpp>
#include <wbt/hard_constraints/wbt_task_reaction_force_lcp_constraint.hpp>

#include <wbt/contacts/wbt_contact_leftfoot.hpp>
#include <wbt/contacts/wbt_contact_rightfoot.hpp>

#include <wbt/optimization_problems/wbt_opt_problem_wbdc.hpp>

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

void test_keyframe_list(){
	sejong::Vect3 desired_keyframe;	
	sejong::Quaternion desired_ori_keyframe;

	Position_KeyFrame pos_kf("HandPos", 4, desired_keyframe);
	Orientation_KeyFrame ori_kf("HandOri", 3, desired_ori_keyframe);

	KeyFrame_List kf_list;
	kf_list.append_keyframe(new Position_KeyFrame("HandPos", 4, desired_keyframe));
	kf_list.append_keyframe(new Orientation_KeyFrame("HandOri", 3, desired_ori_keyframe));


	sejong::Vector q_state_empty;
	double serror = 0.0;
	std::cout << "[WBT] Testing Keyframe container retrieval" << std::endl;
	for (size_t i = 0; i < kf_list.get_size(); i++){
		std::cout << "  KeyFrame: " << i << " name: " << kf_list.get_keyframe(i)->keyframe_name << std::endl;	 
		std::cout << "              link id: " << kf_list.get_keyframe(i)->get_link_id() << std::endl;	 		
		kf_list.get_keyframe(i)->get_slerp_error(q_state_empty, serror);
		std::cout << "          test serror: " << serror << std::endl;	 				
	}

}

void test_wbt_contacts(){
	std::cout << "Testing Contacts" << std::endl;
	LeftFoot_Contact lfc;
	std::cout << "Contact name: " << lfc.contact_name << std::endl;	

	RightFoot_Contact rfc;
	std::cout << "Contact name: " << rfc.contact_name << std::endl;	


}

void test_wbdc_opt_prob(){
	std::cout << "[Main] Testing WBDC Opt Problem Instantiation" << std::endl;
	WBDC_Opt wbdc_test_prob;

	std::cout << "[Main] WBDC_OPT object constructed" << std::endl;

	std::vector<double> F_eval;
	wbdc_test_prob.compute_F(F_eval);
	std::cout << "[Main] Size of F_eval:" << F_eval.size() << std::endl;

	std::vector<double> G_eval;
	std::vector<int> iGfun;
	std::vector<int> jGvar;	
	int neG = 0;
	wbdc_test_prob.compute_G(G_eval, iGfun, jGvar, neG);

	std::cout << "[Main] Size of (G, iG, jG)" << "(" << G_eval.size() 
											  << "," << iGfun.size()
											  << "," << jGvar.size() 
											  << ")" << std::endl;

	std::vector<double> A;
	std::vector<int> iAfun;
	std::vector<int> jAvar;	
	int neA = 0;
	wbdc_test_prob.compute_A(A, iAfun, jAvar, neA);


	double result_test = 0.0;
	wbdc_test_prob.compute_F_objective_function(result_test);	
}


void test_opt_prob_ptr(){
	std::cout << "[Main] Testing WBDC Opt Ptr "<< std::endl;
	Optimization_Problem_Main* ptr_opt_problem = new WBDC_Opt();

	int ObjRow = -1;
	ptr_opt_problem->get_F_obj_Row(ObjRow);
	std::cout << "[Main] Optimization name:" << ptr_opt_problem->problem_name << std::endl;
	std::cout << "[Main] Obj Function has index:" << ObjRow << std::endl;

	delete ptr_opt_problem;
}

void test_contact_wrench_lcp_constraint(){
	std::cout << "[Main] initializing Contact List" << std::endl;
	Contact_List contact_list;
  	contact_list.append_contact(new LeftFoot_Contact());
  	contact_list.append_contact(new RightFoot_Contact());

  	int left_foot_index = 0;
	Contact_Wrench_Constraint cwc(&contact_list, left_foot_index, 0.8, 0.1, 0.05);	

	Contact_LCP_Constraint clcp(&contact_list, left_foot_index);	
}

void test_task_reaction_force_lcp_constraint(){
	Task_Reaction_Force_LCP_Constraint trf_lcp;

}

int main(int argc, char **argv)
{
	std::cout << "[Main] Testing object and argument calls" << std::endl;

	test_wbt_opt_variable();
	test_wholebody_task_objects();
	test_wbt_keyframe();
	test_keyframe_list();

	test_wbt_contacts();

	test_wbdc_opt_prob();
	std::cout << "[Main] WBDC_OPT object destroyed" << std::endl;

	test_opt_prob_ptr();
	test_contact_wrench_lcp_constraint();
	test_task_reaction_force_lcp_constraint();

	return 0;
}