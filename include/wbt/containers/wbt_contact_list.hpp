#ifndef WBT_CONTACT_LIST_H
#define WBT_CONTACT_LIST_H

#include <vector>
#include <wbt/contacts/wbt_contact_main.hpp>

class Contact_List{
public:
	Contact_List();
	~Contact_List();	

	void append_task(Contact* whole_body_contact);
	void get_task_list_copy(std::vector<Contact*>& contact_list_out);

	int get_size();
	Contact* get_contact(int index);

private:
	sejong::Matrix Q_matrix; // contact reaction force cost matrix
	std::vector<Contact*> contact_list;
};

#endif