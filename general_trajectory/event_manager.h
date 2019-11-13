#pragma once
#include "stdafx.h"
#include "event.h"
class EventManager
{
public:
	EventManager();
	~EventManager();

	void Load(std::string time_sequence_file);
	void Process(double t, three_degree_trajectory* tj);

private:
	std::vector<Event> events_;

};

