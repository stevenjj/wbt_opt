#include <wbt/containers/wbt_opt_variable_list.hpp>

#include <iostream>
WBT_Opt_Variable_List::WBT_Opt_Variable_List(){}
WBT_Opt_Variable_List::~WBT_Opt_Variable_List(){
	for(size_t i = 0; i < opt_var_list.size(); i++){
		delete opt_var_list[i];
	}
	opt_var_list.clear();
}

void WBT_Opt_Variable_List::append_variable(WBT_Opt_Variable* opt_variable){
	opt_var_list.push_back(opt_variable);
}


int WBT_Opt_Variable_List::get_size(){
	return opt_var_list.size();
}

WBT_Opt_Variable* WBT_Opt_Variable_List::get_opt_variable(const int index){
	if ((index >= 0) && (index < opt_var_list.size())){
		return opt_var_list[index];
	}else{
		std::cerr << "Error retrieving optimization variable. Index is out of bounds" << std::endl;
		throw "invalid_index";
	}
}