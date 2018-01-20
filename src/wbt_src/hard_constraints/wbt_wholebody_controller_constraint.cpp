#include <wbt/hard_constraints/wbt_wholebody_controller_constraint.hpp>

Wholebody_Controller_Constraint::Wholebody_Controller_Constraint(){
	robot_model = RobotModel::GetRobotModel();
	constraint_name = "WBC constraint";
}
Wholebody_Controller_Constraint::Wholebody_Controller_Constraint(WholeBody_Task_List* wb_task_list_input){
	robot_model = RobotModel::GetRobotModel();
	set_task_list(wb_task_list_input);
	constraint_name = "WBC constraint";
	
}

Wholebody_Controller_Constraint::~Wholebody_Controller_Constraint(){
	std::cout << "WCC destructor called" << std::endl;
}


void Wholebody_Controller_Constraint::set_task_list(WholeBody_Task_List* wb_task_list_input){
	std::cout << "[WBC Constraint] Processing Task List" << std::endl;

	wb_task_list = wb_task_list_input;	
	task_dim = wb_task_list->get_size();

	std::cout << "[WBC Constraint] Task List Processed" << std::endl;
	std::cout << "[WBC Constraint] Task list size: " << task_dim << std::endl;
}

