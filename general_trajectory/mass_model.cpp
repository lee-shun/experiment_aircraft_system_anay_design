#include "stdafx.h"
#include "mass_model.h"
#include "utils.h"
#include "../utility.h"

void NormalMassFlowRateModel::Load(std::map<std::string, std::string> n2vs) {
	std::for_each(n2vs.begin(), n2vs.end(), [this](std::pair<std::string, std::string> it) {
		if (it.first == "ts") {
			this->ts_= convert2doubles(it.second.c_str());
		}
		else if (it.first == "mcs") {
			this->mcs_= convert2doubles(it.second.c_str());
		}
		else {
			printf("NormalMassFlowRateModel, δ֪������:%s\n", it.first.c_str());
			prompt_exit(1);
		}
	});
}

void NormalMassModel::Load(std::map<std::string, std::string> n2vs) {
	//std::for_each(n2vs.begin(), n2vs.end(), [this, ms](std::pair<std::string, std::string> it) {
	for(auto it : n2vs){
		if (it.first == "ts") {
			this->ts_ = convert2doubles(it.second.c_str());
		}
		else if (it.first == "ms") {
			this->ms_ = convert2doubles(it.second.c_str());
		}
		else {
			printf("NormalMassModel, δ֪������:%s\n", it.first.c_str());
			prompt_exit(1);
		}
	}

	if (ts_.size() == 0) {
		printf("ʱ�����в���Ϊ��\n" );
		prompt_exit(1);
	}
	if(ts_.size() != ms_.size()){
		printf("ʱ�����к���������������һ��\n" );
		prompt_exit(1);
	}

}

MassFlowRateModel* MassFlowRateModel::ConstructFromFile(double t_start, const char* filename) {
	std::map<std::string, std::string> name_values;
	split_file(filename, '=', name_values);

	if (name_values["type"] == "NormalMassFlowRateModel") {
		NormalMassFlowRateModel* model = new NormalMassFlowRateModel(t_start);
		name_values.erase("type");
		model->Load(name_values);
		return model;
	}
	else if (name_values["type"] == "NormalMassModel") {
		NormalMassModel* model = new NormalMassModel(t_start);
		name_values.erase("type");
		model->Load(name_values);
		return model;
	}
	else {
		printf("δ֪������ģ�����ƣ� %s\n", name_values["type"].c_str());
		prompt_exit(1);
		return NULL;
	}


}
