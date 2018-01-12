#ifndef WBT_KEYFRAME_LIST_H
#define WBT_KEYFRAME_LIST_H

#include <vector>
#include <wbt/soft_constraints/wbt_keyframe.hpp>

class KeyFrame_List{
public:
	KeyFrame_List();
	~KeyFrame_List();	

	void append_keyframe(KeyFrame* keyframe_obj);
	void get_keyframe_list_copy(std::vector<KeyFrame*>& keyframe_list_out);

	int get_size();
	KeyFrame* get_keyframe(int index);

private:
	sejong::Matrix R_matrix; // keyframe cost matrix
	std::vector<KeyFrame*> keyframe_list;
};

#endif