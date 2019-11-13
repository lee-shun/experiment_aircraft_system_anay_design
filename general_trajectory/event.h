#pragma once
//事件模型（包括弹道初始化）
#include "stdafx.h"

class three_degree_trajectory;


class Event {
public:
	double t_;
//	std::string name_;

public:
	Event(double t) {
		t_ = t;
		//name_ = name;
	}
	void Load(std::string event_define_file);
	void Process(three_degree_trajectory* tj);

private:
	//std::map<std::string, std::string> models_;
	std::vector<std::pair<std::string, std::string>> models_;
};

//class trajectory_initial {
//public:
//	void load(std::string filename) {
//
//	}
//	void initial(three_degree_trajectory* tj) {
//
//	}
//
//private:
//	//state_model initial_state;
//};
