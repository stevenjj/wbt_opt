#include <wbt/soft_constraints/wbt_keyframe_orientation.hpp>

Orientation_KeyFrame::Orientation_KeyFrame(){
	robot_model_ = RobotModel::GetRobotModel();
}

Orientation_KeyFrame::	Orientation_KeyFrame(std::string _keyframe_name, int _link_id, sejong::Quaternion _desired_orientation){
	robot_model_ = RobotModel::GetRobotModel();
	keyframe_name = _keyframe_name;
	link_id = _link_id;
	desired_orientation = _desired_orientation;
}

Orientation_KeyFrame::~Orientation_KeyFrame(){}

int Orientation_KeyFrame::get_link_id(){
	return link_id;
}