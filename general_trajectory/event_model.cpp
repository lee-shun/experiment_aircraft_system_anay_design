//事件模型（包含弹道初始化）
#include "stdafx.h"
#include "event.h"
#include "three_degree_trajectory.h"
#include "utils.h"
#include "rotation_inertia_model.h"








class state_model {
public:
	state_model(std::string filename) {
		state_file_ = filename;
	}
	void process(three_degree_trajectory* tj) {
		std::map<std::string, std::string> name_values;
		split_file(state_file_, '=', name_values);

		for (auto it = name_values.begin(); it != name_values.end(); it++) {
			std::string name = it->first;
			double value = atof(it->second.c_str());
			if (name == "V")
				tj->_V = value;
			else if (name == "Theta")
				tj->_Theta = value / RAD;
			else if (name == "PsiV")
				tj->_Psi_V = value / RAD;
			else if (name == "X")
				tj->_X = value;
			else if (name == "Y")
				tj->_Y = value + tj->platform_altitude_; //初始化导弹Y坐标时，加上发射平台高度
			else if (name == "Z")
				tj->_Z= value;
			else if (name == "Mass")
				tj->_Mass = value;
			//else if (name == "Jx")
			//	tj->Jx= value;
			//else if (name == "Jy")
			//	tj->Jy= value;
			//else if (name == "Jz")
			//	tj->Jz= value;
			else {
				printf("状态初始化文件：《%s》中包含无法识别的状态名： %s ", state_file_.c_str(), name.c_str());
			}

		}
	}
private:
	std::string state_file_;
};

void Event::Load(std::string event_define_file) {

	//split_file(event_define_file, '=', models_);

	split_file_pair_vector(event_define_file, '=', models_);

}

extern double g_step;
void Event::Process(three_degree_trajectory* tj) {

	for (auto it = models_.begin(); it != models_.end(); it++) {
		std::string model_name = it->first;
		std::string model_data = it->second;

		if (model_name == "platform_altitude") {
			tj->platform_altitude_ = atof(model_data.c_str());
		}
		else if (model_name == "states") {
			state_model state(model_data);
			state.process(tj);
		}
		else if (model_name == "step") { //积分步长
			g_step = atof(model_data.c_str());
		}
		else if (model_name == "ratation_inertia") {
			if (tj->_rotation_inertia)
				delete tj->_rotation_inertia;
			tj->_rotation_inertia = RotationInertiaModel::ConstructRotationInertiaModelFromFile(t_, model_data);

		}
		else if (model_name == "aero_data") {//保留原始形式的aero_data模型
			if (tj->_aero_model)
				delete tj->_aero_model;
			tj->_aero_model = new NormalAerodynamicModel(model_data.c_str());
			//tj->aero_data = aerodynamic_open(model_data.c_str());
		}
		else if (model_name == "aero_model") {
			if (tj->_aero_model)
				delete tj->_aero_model;
			tj->_aero_model = AerodynamicModel::ConstructFromFile(t_, model_data.c_str());
			//tj->aero_data = aerodynamic_open(model_data.c_str());
		}
		else if (model_name == "mass_model") {
			if (tj->_mass_model)
				delete tj->_mass_model;
			if (model_data == "0")
				tj->_mass_model = new FixedMassFlowRateModel(0);
			else tj->_mass_model = MassFlowRateModel::ConstructFromFile(t_, model_data.c_str());

		}
		else if (model_name == "thrust_model") {
			if (tj->_thrust_model)
				delete tj->_thrust_model;
			if (model_data == "0")
				tj->_thrust_model = new ZeorThrustModel();
			else tj->_thrust_model = ThrustModel::ConstructFromFile(t_, model_data.c_str());
		}
		else if (model_name == "ideal_control_model") {
			tj->_ideal_control_model = IdealControlModel::ConstructIdealModelFromFile(t_, model_data, tj);
		}
		else {
			printf("警告： 未知模型名称（事件条件）： %s=%s\n", model_name.c_str(), model_data.c_str());
		}

	}
}

