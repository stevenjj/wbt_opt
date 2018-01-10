#ifndef WBT_WHOLEBODY_TASK_LIST_H
#define WBT_WHOLEBODY_TASK_LIST_H

#include <vector>
#include <wbt/tasks/wbt_task_main.hpp>

class WholeBody_Task_List{
public:
	WholeBody_Task_List();
	~WholeBody_Task_List();	

	void append_task(Task* whole_body_task);
	void get_task_list_copy(std::vector<Task*>& task_list_out);

	int get_size();
	Task* get_task(int index);

private:
	std::vector<Task*> task_list;
};

#endif