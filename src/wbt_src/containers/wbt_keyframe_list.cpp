#include <wbt/containers/wbt_keyframe_list.hpp>

#include <iostream>
KeyFrame_List::KeyFrame_List(){}
KeyFrame_List::~KeyFrame_List(){
	for(size_t i = 0; i < keyframe_list.size(); i++){
		delete keyframe_list[i];
	}
	keyframe_list.clear();
}

void KeyFrame_List::append_keyframe(KeyFrame* keyframe_obj){
	keyframe_list.push_back(keyframe_obj);
}

void KeyFrame_List::get_keyframe_list_copy(std::vector<KeyFrame*>& keyframe_list_out){
	keyframe_list_out = keyframe_list;
}

int KeyFrame_List::get_size(){
	return keyframe_list.size();
}

KeyFrame* KeyFrame_List::get_keyframe(int index){
	if ((index >= 0) && (index < keyframe_list.size())){
		return keyframe_list[index];
	}else{
		std::cerr << "Error retrieving keyframe. Index is out of bounds" << std::endl;
		throw "invalid_keyframe_index";
	}
}