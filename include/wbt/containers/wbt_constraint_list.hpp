#ifndef WBT_CONSTRAINT_LIST_H
#define WBT_CONSTRAINT_LIST_H

#include <wbt/hard_constraints/wbt_constraint_main.hpp>
#include <vector>
#include <string>

class Constraint_List{
public:
	Constraint_List();
	~Constraint_List();	

	void append_constraint(const Constraint_Function& constraint);	

	void get_constraint_list_copy(std::vector<Constraint_Function>& constraint_list_out);

	int get_size();
	Constraint_Function* get_constraint(int index);

private:
	std::vector<Constraint_Function> constraint_list;

	std::vector<Constraint_Function*> constraint_list_ptr;	

};

#endif