#ifndef WBT_KEYFRAME_ORIENTATION_H
#define WBT_KEYFRAME_ORIENTATION_H

#include <wbt/soft_constraints/wbt_keyframe.hpp>
#include "RobotModel.hpp"

class Orientation_KeyFrame: public KeyFrame{
public:
	Orientation_KeyFrame();
	Orientation_KeyFrame(std::string _keyframe_name, int link_id, sejong::Quaternion _desired_orientation);
	~Orientation_KeyFrame();

	int  get_link_id();
	void get_diff_error(const sejong::Vector& q_state, sejong::Vector& keyframe_error);
	double get_slerp_error(const sejong::Vector& q_state);	

private:
	int link_id = -1;
	RobotModel* robot_model_;	
	sejong::Quaternion desired_orientation;

	void get_current_orientation(const sejong::Vector& q_state, sejong::Quaternion& current_keyframe_orientation);
};
#endif