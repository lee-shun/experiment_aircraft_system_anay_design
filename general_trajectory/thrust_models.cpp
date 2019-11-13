#include "stdafx.h"
#include "thrust_models.h"
#include "utils.h"
#include "../utility.h"

void LoadNormalThrustModel(NormalThrustModel* model, std::map<std::string, std::string> n2vs) {
	std::for_each(n2vs.begin(), n2vs.end(), [model](std::pair<std::string, std::string> it) {
		if (it.first == "ts") {
			model->ts_ = convert2doubles(it.second.c_str());
		}
		else if (it.first == "thrusts") {
			model->thrusts_ = convert2doubles(it.second.c_str());
		}
		else {
			printf("NormalThrustModel, 未知条件名:%s\n", it.first.c_str());
			prompt_exit(1);
		}
	});
}

ThrustModel* ThrustModel::ConstructFromFile(double t_start, const char* filename) {
	std::map<std::string, std::string> name_values;
	split_file(filename, '=', name_values);

	if (name_values["type"] == "NormalThrustModel") {
		NormalThrustModel* model = new NormalThrustModel(t_start);
		name_values.erase("type");
		LoadNormalThrustModel(model, name_values);
		return model;
	}

	return NULL;
}
