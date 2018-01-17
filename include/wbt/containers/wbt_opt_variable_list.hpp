#ifndef WBT_OPT_VARS_LIST_H
#define WBT_OPT_VARS_LIST_H

#include <vector>
#include <wbt/containers/wbt_opt_variable.hpp>

class WBT_Opt_Variable_List{
public:
	WBT_Opt_Variable_List();
	~WBT_Opt_Variable_List();	

	void append_variable(WBT_Opt_Variable* opt_variable);

	int get_size();
	WBT_Opt_Variable* get_opt_variable(int index);

private:
	std::vector<WBT_Opt_Variable*> opt_var_list;
};

#endif