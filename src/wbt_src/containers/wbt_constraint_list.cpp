#include <wbt/containers/wbt_constraint_list.hpp>

#include <iostream>
Constraint_List::Constraint_List(){}
Constraint_List::~Constraint_List(){}


void Constraint_List::append_constraint(const Constraint_Function& constraint){
	constraint_list.push_back(constraint);	
}

int Constraint_List::get_size(){
	return constraint_list.size();
}

Constraint_Function* Constraint_List::get_constraint(const int index){
	if ((index >= 0) && (index < constraint_list.size())){
		return &constraint_list[index];
	}else{
		std::cerr << "Error retrieving constraint. Index is out of bounds" << std::endl;
		throw "invalid_index";
	}
}