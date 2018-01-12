#ifndef WBT_KEYFRAME_POSITION_H
#define WBT_KEYFRAME_POSITION_H

#include <wbt/soft_constraints/wbt_keyframe.hpp>
#include "RobotModel.hpp"

class Position_KeyFrame: public KeyFrame{
public:
	Position_KeyFrame();
	Position_KeyFrame(std::string _keyframe_name, int link_id, sejong::Vect3 _desired_position);
	~Position_KeyFrame();

	int  get_link_id();
	void get_error(const sejong::Vector& q_state, sejong::Vect3& keyframe_error);

private:
	int link_id = -1;
	RobotModel* robot_model_;	
	sejong::Vect3 desired_position;

	void get_current_position(const sejong::Vector& q_state, sejong::Vect3& current_keyframe_position);
};
#endif