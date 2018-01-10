#ifndef WBT_OPT_VARS_H
#define WBT_OPT_VARS_H

#include <string>
#include <wbt/optimization_constants.hpp>

class WBT_Opt_Variable{
public:
	std::string type = "no_type";
	std::string name = "undefined_name";
	double 		value = 0.0;
	int			time_step = -1;
	int 		index = -1;

	double l_bound = -INFINITY;
	double u_bound = INFINITY;	

	// Constructors
	WBT_Opt_Variable();
	WBT_Opt_Variable(std::string _name, double _value);

	// Destructors
	~WBT_Opt_Variable();	
};

#endif