#include "stdafx.h"

#include "../utility.h"

#include "rotation_inertia_model.h"

RotationInertiaModel* RotationInertiaModel::ConstructRotationInertiaModelFromFile(double t_start, std::string filename) {
	std::map<std::string, std::string> name_values;
	split_file(filename, '=', name_values);

	if (name_values["type"] == "FixedRotationInertiaModel") {
		FixedRotationInertiaModel* model = new FixedRotationInertiaModel();
		name_values.erase("type");
		LoadFixedRotationInertiaModel(model, name_values);
		return model;
	}
	else if (name_values["type"] == "GeneralRotationInertiaModel") {
		GeneralRotationInertiaModel* model = new GeneralRotationInertiaModel(t_start);
		name_values.erase("type");
		LoadGeneralRotationInertiaModel(model, name_values);
		return model;
	}
	else {
		printf("《%s》文件中暂时不支持的转动惯量模型： %s \n", filename.c_str(), name_values["type"].c_str());
		prompt_exit(1);
		return NULL;
	}
}

void LoadFixedRotationInertiaModel(FixedRotationInertiaModel* model, Name2Value& n2vs) {
	std::for_each(n2vs.begin(), n2vs.end(), [model](std::pair<std::string, std::string> it) {
		if (it.first == "JX") {
			model->JX_ = atof(it.second.c_str());
		}
		else if (it.first == "JY") {
			model->JY_ = atof(it.second.c_str()); 
		}
		else if (it.first == "JZ") {
			model->JZ_ = atof(it.second.c_str());
		}
		else {
			printf("FixedRotationInertiaModel, 未知条件名:%s\n", it.first.c_str());
			prompt_exit(1);
		}
	});

}

void LoadGeneralRotationInertiaModel(GeneralRotationInertiaModel* model,  Name2Value& n2vs) {
	std::for_each(n2vs.begin(), n2vs.end(), [model](std::pair<std::string, std::string> it) {
		if (it.first == "ts") {
			model->ts_ = convert2doubles(it.second);
		}
		else if (it.first == "JXs") {
			model->JXs_ = convert2doubles(it.second);
		}
		else if (it.first == "JYs") {
			model->JYs_ = convert2doubles(it.second);
		}
		else if (it.first == "JZs") {
			model->JZs_ = convert2doubles(it.second);
		}
		else {
			printf("GeneralRotationInertiaModel, 未知条件名:%s\n", it.first.c_str());
			prompt_exit(1);
		}
	});

}