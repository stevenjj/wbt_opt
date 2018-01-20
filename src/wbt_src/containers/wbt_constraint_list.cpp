#include <wbt/containers/wbt_constraint_list.hpp>

#include <iostream>
Constraint_List::Constraint_List(){}
Constraint_List::~Constraint_List(){
	for(size_t i = 0; i < constraint_list.size(); i++){
		delete constraint_list[i];
	}
	constraint_list.clear();	
}


void Constraint_List::append_constraint(Constraint_Function* constraint){
	constraint_list.push_back(constraint);	
}

int Constraint_List::get_size(){
	return constraint_list.size();
}

Constraint_Function* Constraint_List::get_constraint(const int index){
	if ((index >= 0) && (index < constraint_list.size())){
		return constraint_list[index];
	}else{
		std::cerr << "Error retrieving constraint. Index is out of bounds" << std::endl;
		throw "invalid_index";
	}
}