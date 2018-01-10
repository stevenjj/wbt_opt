#include <wbt/containers/wbt_wholebody_task_list.hpp>

#include <iostream>
WholeBody_Task_List::WholeBody_Task_List(){}
WholeBody_Task_List::~WholeBody_Task_List(){}

void WholeBody_Task_List::append_task(Task* whole_body_task){
	task_list.push_back(whole_body_task);
}

void WholeBody_Task_List::get_task_list_copy(std::vector<Task*>& task_list_out){
	task_list_out = task_list;
}