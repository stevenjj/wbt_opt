#ifndef WBT_OPT_VARS_H
#define WBT_OPT_VARS_H

#include <string>
#include <wbt/optimization_constants.hpp>

class WBT_Opt_Variable{
public:
	int type = VAR_TYPE_NONE;
	std::string name = "undefined_name";
	double 		value = 0.0;
	int			time_step = -1;
	int 		index = -1;

	double l_bound = -OPT_INFINITY;
	double u_bound = OPT_INFINITY;	

	// Constructors
	WBT_Opt_Variable();
	WBT_Opt_Variable(std::string _name, double _value);
	WBT_Opt_Variable(std::string _name, double _value, double _l_bound, double _u_bound);	
	WBT_Opt_Variable(std::string _name, int _time_step, double _value, double _l_bound, double _u_bound);	
	WBT_Opt_Variable(std::string _name, int _type, int _time_step, double _value, double _l_bound, double _u_bound);		

	// Destructors
	~WBT_Opt_Variable();	
};

#endif