#include <wbt/soft_constraints/wbt_keyframe_position.hpp>

Position_KeyFrame::Position_KeyFrame(){
	robot_model_ = RobotModel::GetRobotModel();
}

Position_KeyFrame::	Position_KeyFrame(std::string _keyframe_name, int _link_id, sejong::Vect3 _desired_position){
	robot_model_ = RobotModel::GetRobotModel();
	keyframe_name = _keyframe_name;
	link_id = _link_id;
	desired_position = _desired_position;
}

Position_KeyFrame::~Position_KeyFrame(){}

void Position_KeyFrame::get_error(const sejong::Vector& q_state, sejong::Vect3& keyframe_error){
}

int Position_KeyFrame::get_link_id(){
	return link_id;
}