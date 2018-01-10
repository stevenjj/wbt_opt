#include <wbt/wbt_opt_variable.hpp>

WBT_Opt_Variable::WBT_Opt_Variable(){}
WBT_Opt_Variable::WBT_Opt_Variable(std::string _name, double _value){
	name = _name;
	value = _value;	
}

WBT_Opt_Variable::~WBT_Opt_Variable(){}