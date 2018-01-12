#ifndef WBT_KEYFRAME_H
#define WBT_KEYFRAME_H

#include <Utils/wrap_eigen.hpp>
#include <string>

class KeyFrame{
public:
	KeyFrame(){}
	virtual ~KeyFrame(){}

	std::string keyframe_name = "Undefined Keyframe";
	int timestep = -1;

	void get_keyframe_name(std::string& keyframe_name_out){
		keyframe_name_out = keyframe_name;
	}


};
#endif