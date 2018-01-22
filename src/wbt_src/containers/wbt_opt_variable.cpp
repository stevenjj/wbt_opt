#include <wbt/containers/wbt_opt_variable.hpp>

WBT_Opt_Variable::WBT_Opt_Variable(){}
WBT_Opt_Variable::WBT_Opt_Variable(std::string _name, double _value){
	name = _name;
	value = _value;	
}

WBT_Opt_Variable::WBT_Opt_Variable(std::string _name, double _value, double _l_bound, double _u_bound){
	name = _name;
	value = _value;	
	l_bound = _l_bound;
	_u_bound = _u_bound;	
}

WBT_Opt_Variable::WBT_Opt_Variable(std::string _name, int _time_step, double _value, double _l_bound, double _u_bound){
	name = _name;
	value = _value;	
	time_step = _time_step;
	l_bound = _l_bound;
	_u_bound = _u_bound;	

}

WBT_Opt_Variable::WBT_Opt_Variable(std::string _name, int _type, int _time_step, double _value, double _l_bound, double _u_bound){
	name = _name;
	type = _type;
	value = _value;	
	time_step = _time_step;
	l_bound = _l_bound;
	_u_bound = _u_bound;		
}

WBT_Opt_Variable::~WBT_Opt_Variable(){}