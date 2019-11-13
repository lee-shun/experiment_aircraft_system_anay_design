#include "stdafx.h"
#include "event_manager.h"
#include "three_degree_trajectory.h"
#include "utils.h"
#include "../utility.h"

using namespace std;
EventManager::EventManager()
{

}

void EventManager::Load(std::string time_sequence_file) {
	char line[MAX_LINE] = "";
	
	FILE* fin = fopen(time_sequence_file.c_str(), "r");
	if (fin == NULL) {
		printf("无法打开文件: %s", time_sequence_file.c_str());
		prompt_exit(1);
	}


	int row = 0;
	while (!feof(fin)) {
		if(0!=fgets(line, MAX_LINE, fin))
		{
			if (line[strlen(line) - 1] == '\n')
				line[strlen(line) - 1] = 0;
			row++;

			if (IsCommentLine(line)) {
				continue;  //跳过注释行
			}
			else {  //非注释行
				try {
					istringstream iss(line);
					double t;
					iss >> t;
					//string event_name;
					//iss >> event_name;
					string event_define_file;
					iss >> event_define_file;

					Event e(t);
					e.Load(event_define_file);
					events_.push_back(e);
				}
				catch (std::exception& e) {
					printf(" 时序文件：《%s》 第%d行 发生异常: %s \n", time_sequence_file.c_str(), row, e.what());
					prompt_exit(1);
				}
			}

		}
	}
	fclose(fin);

	std::sort(events_.begin(), events_.end(), 
		[](const Event &a, const Event& b) -> bool	{	return a.t_ < b.t_;	}
	);

}

EventManager::~EventManager()
{
}

void EventManager::Process(double t, three_degree_trajectory* tj) {
	//if (t >= 39.999) {
	//	int i = 0;
	//}

	for (auto it = events_.begin(); it != events_.end(); ) {
		if (fabs(it->t_ - t) < 0.00001) //步长不可能小于0.0001，因此此精度下认为时间相等，事件触发
		{
			it->Process(tj);
			it = events_.erase(it);
		}
		else it++;
	}
	
	


	//auto bounds = std::equal_range(events_.begin(), events_.end(), Event(t), 
	//	[](const Event &a, const Event &b) -> bool {
	//		return a.t_ < b.t_ ;	
	//		}
	//);
	//for (auto it = bounds.first; it != bounds.second; it++) {
	//	it->Process(tj);
	//}
	//
	////已经处理过的事件，从事件列表中清除，以节省时间
	//events_.erase(bounds.first, bounds.second);

}
