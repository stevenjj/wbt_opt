#ifndef WBT_KEYFRAME_H
#define WBT_KEYFRAME_H

#include <Utils/wrap_eigen.hpp>
#include <string>
#include <iostream>
class KeyFrame{
public:
	KeyFrame(){}
	virtual ~KeyFrame(){}
	virtual void get_keyframe_name(std::string& keyframe_name_out){
		keyframe_name_out = keyframe_name;
	}

	virtual int  get_link_id(){return -1;}
	virtual void get_error(const sejong::Vector& q_state, sejong::Vect3& keyframe_error){}
	virtual void get_diff_error(const sejong::Vector& q_state, sejong::Vector& keyframe_error){}
	virtual void get_slerp_error(const sejong::Vector& q_state, double& error){}

	std::string keyframe_name = "Undefined Keyframe";
	int timestep = -1;


};
#endif