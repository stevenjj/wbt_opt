#include <wbt/containers/wbt_contact_list.hpp>

#include <iostream>
Contact_List::Contact_List(){}
Contact_List::~Contact_List(){
	for(size_t i = 0; i < contact_list.size(); i++){
		delete contact_list[i];
	}
	contact_list.clear();
}

void Contact_List::append_task(Contact* whole_body_contact){
	contact_list.push_back(whole_body_contact);
}

void Contact_List::get_task_list_copy(std::vector<Contact*>& contact_list_out){
	contact_list_out = contact_list;
}

int Contact_List::get_size(){
	return contact_list.size();
}

Contact* Contact_List::get_contact(int index){
	if ((index >= 0) && (index < contact_list.size())){
		return contact_list[index];
	}else{
		std::cerr << "Error retrieving Contact. Index is out of bounds" << std::endl;
		throw "invalid_index";
	}
}