#ifndef WBT_CONSTRAINT_LIST_H
#define WBT_CONSTRAINT_LIST_H

#include <wbt/hard_constraints/wbt_constraint_main.hpp>
#include <vector>
#include <string>

class Constraint_List{
public:
	Constraint_List();
	~Constraint_List();	

	void append_constraint(Constraint_Function* constraint);	

	void get_constraint_list_copy(std::vector<Constraint_Function*>& constraint_list_out);

	int get_size();

	int get_num_constraint_funcs();
	Constraint_Function* get_constraint(int index);

private:
	std::vector<Constraint_Function*> constraint_list;
	int num_constraint_funcs = 0;

};

#endif