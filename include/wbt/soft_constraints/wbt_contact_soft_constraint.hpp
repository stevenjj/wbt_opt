#ifndef WBT_CONTACT_SOFT_CONSTRAINT_H
#define WBT_CONTACT_SOFT_CONSTRAINT_H

#include <Utils/wrap_eigen.hpp>
#include <string>
#include <iostream>
class Contact_Soft_Constraint{
public:
	Contact_Soft_Constraint(){}
	virtual ~Contact_Soft_Constraint(){}

	virtual int  get_link_id(){return -1;}

	std::string contact_name = "Undefined Contact";

};
#endif