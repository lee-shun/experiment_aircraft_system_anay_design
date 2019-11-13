//�������ģ�ͣ���Ӧ�ķ����飩
#include "stdafx.h"
#include "ideal_control_models.h"
#include "three_degree_trajectory.h"
#include "utils.h"
#include "../utility.h"


std::string dump_array(std::vector<double> t_array)
{
	std::stringstream ss;
	for (auto it = t_array.begin(); it != t_array.end(); it++) {
		ss << *it << " ";
	}
	return ss.str();
	
}
//ʱ���������������
void verify_t_array(std::vector<double> t_array)
{
	for (size_t i = 0; i < t_array.size() - 1; ++i) {
		if (t_array[i] > t_array[i + 1]) {
			printf("������ʱ������[%s]������˲���ٽ��м���\n", dump_array(t_array).c_str());
			prompt_exit(1);
		}
	}
}


bool MovementLaw::Load(const char* filename, three_degree_trajectory* tj) {
	this->platform_altitude_ = tj->platform_altitude_;

	std::ifstream fi(filename);
	if (!fi)
	{
		printf("�޷����˶������ļ�%s\n", filename);
		return false;
	}

	string line;

	while (std::getline(fi, line))
	{
		ltrim(line);

		if (line.size() == 0)//����
			continue;
		if (line[0] == '#') //ע����
			continue;

		std::istringstream iss(line);
		vector<double> row{ istream_iterator<double>{iss},
			istream_iterator<double>{} };

		if (row.size() != 4) {
			printf("%s�ļ��д���������У�\n %s\n", filename, line.c_str());
			return false;
		}

		t_s.push_back(row[0]);
		x_s.push_back(row[1]);
		y_s.push_back(row[2]);
		z_s.push_back(row[3]);

	}
	return true;

}

IdealControlModel* IdealControlModel::ConstructIdealModelFromFile(double t, std::string filename, three_degree_trajectory* tj) {
	std::map<std::string, std::string> name_values;
	split_file(filename, '=', name_values);

	if (name_values["type"] == "AlphaBetaGammaV_Ideal") {
		AlphaBetaGammaV_ControlModel* model = new AlphaBetaGammaV_ControlModel(t);
		name_values.erase("type");
		model->Load(name_values);
		return model;
	}
	else if (name_values["type"] == "ProNav_Ideal") {
		Proportional_Navigation_ControlModel* model = new Proportional_Navigation_ControlModel(t);
		name_values.erase("type");
		model->Load(name_values, tj);
		return model;
	}
	else if (name_values["type"] == "ThreePointNav_Ideal") {
		//Threepoint_Navigation_ControlModel* model = new Threepoint_Navigation_ControlModel(t);
		//!!!!��ʱʹ��ǰ�õ�����С�߶����㷨����ǰ�ý���Ϊ0���������㷨��������ΪĿǰ���㷨��ʵ����û�м��������غͲ໬��
		Qianzhi_Navigation_ControlModel* model = new Qianzhi_Navigation_ControlModel(t);
		model->d_epsilon0_h = 0;
		model->d_epsilon0_v = 0;
		name_values.erase("type");
		model->Load(name_values, tj);
		return model;
	}
	else if (name_values["type"] == "GaofeiThreePointNav_Ideal") {
		Gaofei_Navigation_ControlModel* model = new Gaofei_Navigation_ControlModel(t);
		name_values.erase("type");
		model->Load(name_values, tj);
		return model;
	}
	else if (name_values["type"] == "QianZhiNav_Ideal") {//ǰ������
		Qianzhi_Navigation_ControlModel* model = new Qianzhi_Navigation_ControlModel(t);
		name_values.erase("type");
		model->Load(name_values, tj);
		return model;
	}


	//else if (name_values["type"] == "PursuitNav_Ideal") { //�ٶ�׷�ٷ�
	//	Pursuit_Navigation_ControlModel* model = new Pursuit_Navigation_ControlModel(t);
	//	name_values.erase("type");
	//	model->Load(name_values);
	//	return model;
	//}
	else if (name_values["type"] == "BestCL_CD_Ideal") {
		BestCL_CD_ControlModel* model = new BestCL_CD_ControlModel(t);
		name_values.erase("type");
		model->Load(name_values);
		return model;
	}
	else if (name_values["type"] == "DeltaYZ_Ideal") {
		DeltaYZ_ControlModel* model = new DeltaYZ_ControlModel(t);
		name_values.erase("type");
		model->Load(name_values);
		return model;
	}
	else if (name_values["type"] == "ThetaPsiV_Ideal") {
		ThetaPsiV_ControlModel* model = new ThetaPsiV_ControlModel(t);
		name_values.erase("type");
		model->Load(name_values);
		return model;
	}
	else if (name_values["type"] == "VarTheta_Psi_Ideal") {
		VarTheta_Psi_ControlModel* model = new VarTheta_Psi_ControlModel(t);
		name_values.erase("type");
		model->Load(name_values);
		return model;
	}
	else if (name_values["type"] == "Ny2Nz2_Ideal") {
		Ny2Nz2_ControlModel* model = new Ny2Nz2_ControlModel(t);
		name_values.erase("type");
		model->Load(name_values);
		return model;
	}
	else if (name_values["type"] == "YZ_Ideal") {
		YZ_ControlModel* model = new YZ_ControlModel(t);
		name_values.erase("type");
		model->Load(name_values);
		return model;
	}
	else if (name_values["type"] == "XYZ_FILE_Ideal") {
		XYZ_ControlModel* model = new XYZ_ControlModel(t);
		name_values.erase("type");
		model->Load(name_values);
		return model;
	}
	else {
		printf("��%s���ļ�����ʱ��֧�ֵ��������ģ�ͣ� %s \n", filename.c_str(), name_values["type"].c_str());
		prompt_exit(1);
		return NULL;
	}

}

void DeltaYZ_ControlModel::control_equations(double t, double dt, double y[], double* delta_y, double* delta_z, double* alpha, double*beta, double* gamma_v, three_degree_trajectory* tj) {
	double V = y[three_degree_trajectory::V_Index];
	//������ƹ��ɣ� ���ƶ�ƫ�Ǻ��ٶ���б��
	*gamma_v = 0;
	double t_interp = t - t_start_; //��ֵʱҪ��ȥ���ʱ��
	if (delta_y_.size() != 0 && delta_y_t_.size() != 0)
		*delta_y = interp11(delta_y_, delta_y_t_, t - t_start_);
	else *delta_y = 0;
	if (delta_z_.size() != 0 && delta_z_t_.size() != 0)
		*delta_z = interp11(delta_z_, delta_z_t_, t - t_start_);
	else *delta_z = 0;

	//��������
	double altitude = y[three_degree_trajectory::Y_Index]; //�����߶�
	double sonic = g_atmosphere_model.GetSonic(altitude);
	//double rho = g_atmosphere_model.GetRHO(altitude);
	double mach = V / sonic;



	//ƽ���ϵʽ�����ݵ�ǰ�Ķ�ƫ�ǣ���ƽ�⹥�ǺͲ໬��

	//��ȡƽ�⹥��ʱ��������һ�εĲ໬�ǣ���Ϊ�໬�Ƕ�����ϵ�����������������أ�Ӱ���С�����Ҳ໬�ǵı仯����һ�����Ҳ�������ģ���˴˴�����С��
	*alpha = tj->_aero_model->GetBalanceAlpha(t, mach, tj->_beta, *delta_z);
	*beta = tj->_aero_model->GetBalanceBeta(t, mach, *alpha, *delta_y);
	//*alpha = GetBalanceAlpha_Rad(tj->aero_data, mach, tj->_beta, *delta_z);
	//*beta = GetBalanceBeta_Rad(tj->aero_data, mach, *alpha, *delta_y);   //

	//*alpha = GetBalanceAlpha_Rad(tj->aero_data, mach, 0, *delta_z);//��ȡƽ�⹥��ʱ������0�໬�ǣ���һ�����
	//*beta = GetBalanceBeta_Rad(tj->aero_data, mach, 0, *delta_y);   //��ȡƽ��໬��ʱ������0���ǣ����ϴ�ԼΪ1/4--1/3
	//fprintf(tj->f_zhixin, "%15.10f %15.10f %15.10f\n", t, y[three_degree_trajectory::Y_Index], y[three_degree_trajectory::Z_Index]);
};

void DivideByRad(std::vector<double>& v) {
	std::transform(v.begin(), v.end(), v.begin(),
		[](double t) {return t / RAD; });
}

void XYZ_ControlModel::Load(Name2Value& n2vs){
	std::for_each(n2vs.begin(), n2vs.end(), [this](std::pair<std::string, std::string> it) {
		if (it.first == "FilePath") {
			string filepath = it.second;
			printf("����·��%s\n", filepath.c_str());
			FILE* fi111;
			double x_test[10000], y_test[10000], z_test[10000];
			fopen_s(&fi111, filepath.c_str(), "r");

			//char s1;
			int p1 = 0, p2 = 0;

			while (fscanf_s(fi111, "%lf %lf %lf\n", &x_test[p1], &y_test[p1], &z_test[p1]) != EOF)
				p1++;

			fclose(fi111);
			for (int test_i = 0; test_i < p1; test_i++)
			{
				this->x_array.push_back(x_test[test_i]);
				this->y_array.push_back(y_test[test_i]);
				this->z_array.push_back(z_test[test_i]);
			}
		}
		
		
	});
	
}

void YZ_ControlModel::Load(Name2Value& n2vs) {
	std::for_each(n2vs.begin(), n2vs.end(), [this](std::pair<std::string, std::string> it) {
		if (it.first == "y_t") {
			this->y_t_array = convert2doubles(it.second);
			verify_t_array(this->y_t_array);
		}
		else if (it.first == "y") {
			this->y_array = convert2doubles(it.second);
		}
		else if (it.first == "z_t") {
			this->z_t_array = convert2doubles(it.second);
			verify_t_array(this->z_t_array);
		}
		else if (it.first == "z") {
			this->z_array = convert2doubles(it.second);
		}
		else {
			printf("YZ_ControlModel, δ֪������:[%s]\n", it.first.c_str());
			prompt_exit(1);
		}
	});

	if (y_t_array.size() != y_array.size()) {
		printf("ʱ�������������е���Ŀ��ƥ��\n");
		prompt_exit(1);
	}

	else return;
}

void ThetaPsiV_ControlModel::Load(Name2Value& n2vs) {
	std::for_each(n2vs.begin(), n2vs.end(), [this](std::pair<std::string, std::string> it) {
		if (it.first == "theta_t") {
			this->theta_t_array = convert2doubles(it.second);
			verify_t_array(this->theta_t_array);
		}
		else if (it.first == "theta") {
			this->theta_array = convert2doubles(it.second);
			DivideByRad(this->theta_array);
		}
		else if (it.first == "psi_v_t") {
			this->psi_v_t_array = convert2doubles(it.second);
			verify_t_array(this->psi_v_t_array);
		}
		else if (it.first == "psi_v") {
			this->psi_v_array = convert2doubles(it.second);
			DivideByRad(this->psi_v_array);
		}
		else {
			printf("ThetaPsiV_ControlModel, δ֪������:[%s]\n", it.first.c_str());
			prompt_exit(1);
		}
	});

	if (theta_t_array.size() != theta_array.size()) {
		printf("ʱ�䵯��������е���Ŀ��ƥ��\n");
		prompt_exit(1);
	}
	if (psi_v_t_array.size() != psi_v_array.size()) {
		printf("ʱ�䵯��������е���Ŀ��ƥ��\n");
		prompt_exit(1);
	}
}

void VarTheta_Psi_ControlModel::Load(Name2Value& n2vs) {
	std::for_each(n2vs.begin(), n2vs.end(), [this](std::pair<std::string, std::string> it) {
		if (it.first == "vartheta_t") {
			this->vartheta_t_array = convert2doubles(it.second);
			verify_t_array(this->vartheta_t_array);
		}
		else if (it.first == "vartheta") {
			this->vartheta_array = convert2doubles(it.second);
			DivideByRad(this->vartheta_array);
		}
		else if (it.first == "psi_t") {
			this->psi_t_array = convert2doubles(it.second);
			verify_t_array(this->psi_t_array);
		}
		else if (it.first == "psi") {
			this->psi_array = convert2doubles(it.second);
			DivideByRad(this->psi_array);
		}
		else {
			printf("ThetaPsiV_ControlModel, δ֪������:[%s]\n", it.first.c_str());
			prompt_exit(1);
		}
	});

	if (vartheta_t_array.size() != vartheta_array.size()) {
		printf("ʱ�丩�������е���Ŀ��ƥ��\n");
		prompt_exit(1);
	}
	if (psi_t_array.size() != psi_array.size()) {
		printf("ʱ��ƫ�������е���Ŀ��ƥ��\n");
		prompt_exit(1);
	}
}

void Ny2Nz2_ControlModel::Load(Name2Value& n2vs) {
	std::for_each(n2vs.begin(), n2vs.end(), [this](std::pair<std::string, std::string> it) {
		if (it.first == "ny2_t") {
			this->ny2_t_array = convert2doubles(it.second);
			verify_t_array(this->ny2_t_array);
		}
		else if (it.first == "ny2") {
			this->ny2_array = convert2doubles(it.second);
		}
		else if (it.first == "nz2_t") {
			this->nz2_t_array = convert2doubles(it.second);
			verify_t_array(this->nz2_t_array);
		}
		else if (it.first == "nz2") {
			this->nz2_array = convert2doubles(it.second);
		}
		else {
			printf("Ny2Nz2_ControlModel, δ֪������:[%s]\n", it.first.c_str());
			prompt_exit(1);
		}
	});

	if (ny2_t_array.size() != ny2_array.size()) {
		printf("ʱ�䷨��������е���Ŀ��ƥ��\n");
		prompt_exit(1);
	}
	if (nz2_t_array.size() != nz2_array.size()) {
		printf("ʱ�����������е���Ŀ��ƥ��\n");
		prompt_exit(1);
	}
}


void DeltaYZ_ControlModel::Load(Name2Value& n2vs) {
	std::for_each(n2vs.begin(), n2vs.end(), [this](std::pair<std::string, std::string> it) {
		if (it.first == "delta_y_t") {
			this->delta_y_t_ = convert2doubles(it.second);
			verify_t_array(this->delta_y_t_);
		}
		else if (it.first == "delta_y") {
			this->delta_y_ = convert2doubles(it.second);
			DivideByRad(this->delta_y_);
		}
		else if (it.first == "delta_z_t") {
			this->delta_z_t_ = convert2doubles(it.second);
			verify_t_array(this->delta_z_t_);
		}
		else if (it.first == "delta_z") {
			this->delta_z_ = convert2doubles(it.second);
			DivideByRad(this->delta_z_);
		}
		else {
			printf("DeltaYZ_ControlModel, δ֪������:[%s]\n", it.first.c_str());
			prompt_exit(1);
		}
	});

	if (delta_y_t_.size() != delta_y_.size()) {
		printf("ƫ����ƫ��ʱ�����е���Ŀ��ƥ��\n");
		prompt_exit(1);
	}
	else if (delta_z_t_.size() != delta_z_.size()) {
		printf("������ƫ��ʱ�����е���Ŀ��ƥ��\n");
		prompt_exit(1);
	}
	else return;
}

void AlphaBetaGammaV_ControlModel::Load(Name2Value& n2vs) {
	std::for_each(n2vs.begin(), n2vs.end(), [this](std::pair<std::string, std::string> it) {
		if (it.first == "alpha_t") {
			this->alpha_t_array = convert2doubles(it.second);
			verify_t_array(this->alpha_t_array);
		}
		else if (it.first == "alpha") {
			this->alpha_array = convert2doubles(it.second);
			std::transform(this->alpha_array.begin(), this->alpha_array.end(), this->alpha_array.begin(),
				[](double t) {return t / RAD; });
		}
		else if (it.first == "beta_t") {
			this->beta_t_array = convert2doubles(it.second);
			verify_t_array(this->beta_t_array);
		}
		else if (it.first == "beta") {
			this->beta_array = convert2doubles(it.second);
			std::transform(this->beta_array.begin(), this->beta_array.end(), this->beta_array.begin(),
				[](double t) {return t / RAD; });
		}
		else if (it.first == "gammav_t") {
			this->gammav_t_array = convert2doubles(it.second);
			verify_t_array(this->gammav_t_array);
		}
		else if (it.first == "gammav") {
			this->gammav_array = convert2doubles(it.second);
			std::transform(this->gammav_array.begin(), this->gammav_array.end(), this->gammav_array.begin(),
				[](double t) {return t / RAD; });
		}
		else {
			printf("AlphaBetaGammaV_ControlModel, δ֪������:%s\n", it.first.c_str());
			prompt_exit(1);
		}
	});

	if (alpha_t_array.size() != alpha_array.size()) {
		printf("����ʱ�����е���Ŀ��ƥ��\n");
		prompt_exit(1);
	}
	else if (beta_t_array.size() != beta_array.size()) {
		printf("�໬��ʱ�����е���Ŀ��ƥ��\n");
		prompt_exit(1);
	}
	else if (gammav_t_array.size() != gammav_array.size()) {
		printf("�ٶ���б��ʱ�����е���Ŀ��ƥ��\n");
	}
	else return;
}

void Proportional_Navigation_ControlModel::Load(Name2Value& n2vs,  three_degree_trajectory* tj) {
	for_each(n2vs.begin(), n2vs.end(), [this, tj](std::pair<std::string, std::string> it) {
		if (it.first == "K") {
			this->K = atof(it.second.c_str());
		}
		//else if (it.first == "x_target") {
		//	this->x_t_0 = atof(it.second.c_str());
		//}
		//else if (it.first == "y_target") {
		//	this->y_t_0 = atof(it.second.c_str());
		//}
		//else if (it.first == "z_target") {
		//	this->z_t_0 = atof(it.second.c_str());
		//}
		//else if (it.first == "v_target_x") {
		//	this->v_t_x = atof(it.second.c_str());
		//}
		//else if (it.first == "v_target_z") {
		//	this->v_t_z = atof(it.second.c_str());
		//}
		else if (it.first == "target_movement_filepath") {
			target_movement_.Load(it.second.c_str(), tj);
			//this->LoadTargetFile();
		}
		else if (it.first == "distance_of_stop_nav") {
			this->distance_of_stop_nav = atof(it.second.c_str());
		}
		else {
			printf("Proportional_Navigation_ControlModel, δ֪������:%s\n", it.first.c_str());
			prompt_exit(1);
		}
	});
}

void Gaofei_Navigation_ControlModel::Load(Name2Value& n2vs, three_degree_trajectory* tj) {
	for_each(n2vs.begin(), n2vs.end(), [this, tj](std::pair<std::string, std::string> it) {
		if (it.first == "gaofei_height") {
			this->Hd = atof(it.second.c_str());
		}
		else if (it.first == "r0") {
			this->R0 = atof(it.second.c_str());
		}
		else if (it.first == "laser_filepath") {
			//this->LoadLaserFile(it.second.c_str());
		}
		else if (it.first == "guidance_movement_filepath") {
			guidance_station_movement_.Load(it.second.c_str(), tj);
		}
		else if (it.first == "target_movement_filepath") {
			target_movement_.Load(it.second.c_str(), tj);
		}
		else {
			printf("Gaofei_Navigation_ControlModel, δ֪������:%s\n", it.first.c_str());
			prompt_exit(1);
		}
	});
}

void Threepoint_Navigation_ControlModel::Load(Name2Value& n2vs, three_degree_trajectory* tj) {
	for_each(n2vs.begin(), n2vs.end(), [this, tj](std::pair<std::string, std::string> it) {
		//if (it.first == "x_target") {
		//	this->x_t_0 = atof(it.second.c_str());
		//}
		//else if (it.first == "y_target") {
		//	this->y_t_0 = atof(it.second.c_str());
		//}
		//else if (it.first == "z_target") {
		//	this->z_t_0 = atof(it.second.c_str());
		//}
		//else if (it.first == "v_target_x") {
		//	this->v_t_x = atof(it.second.c_str());
		//}
		//else if (it.first == "v_target_z") {
		//	this->v_t_z = atof(it.second.c_str());
		//}
		//else if (it.first == "x_control") {
		//	this->x_c_0 = atof(it.second.c_str());
		//}
		//else if (it.first == "y_control") {
		//	this->y_c_0 = atof(it.second.c_str());
		//}
		//else if (it.first == "z_control") {
		//	this->z_c_0 = atof(it.second.c_str());
		//}
		//else if (it.first == "v_control_x") {
		//	this->v_c_x = atof(it.second.c_str());
		//}
		//else if (it.first == "v_control_y") {
		//	this->v_c_y = atof(it.second.c_str());
		//}
		//else if (it.first == "v_control_z") {
		//	this->v_c_z = atof(it.second.c_str());
		//}
		/*if (it.first == "distance_of_stop_nav") {
			this->distance_of_stop_nav = atof(it.second.c_str());
		}*/
		if (it.first == "guidance_movement_filepath") {
			guidance_station_movement_.Load(it.second.c_str(), tj);
			//this->LoadGuidanceFile(it.second.c_str());
		}
		else if (it.first == "target_movement_filepath") {
			target_movement_.Load(it.second.c_str(), tj);
			//this->LoadTargetFile(it.second.c_str());
		}
		else {
			printf("Threepoint_Navigation_ControlModel, δ֪������:%s\n", it.first.c_str());
			prompt_exit(1);
		}
	});
}

void Qianzhi_Navigation_ControlModel::Load(Name2Value& n2vs, three_degree_trajectory* tj) {
	for_each(n2vs.begin(), n2vs.end(), [this, tj](std::pair<std::string, std::string> it) {
		//if (it.first == "x_target") {
		//	this->x_t_0 = atof(it.second.c_str());
		//}
		//else if (it.first == "y_target") {
		//	this->y_t_0 = atof(it.second.c_str());
		//}
		//else if (it.first == "z_target") {
		//	this->z_t_0 = atof(it.second.c_str());
		//}
		//else if (it.first == "v_target_x") {
		//	this->v_t_x = atof(it.second.c_str());
		//}
		//else if (it.first == "v_target_z") {
		//	this->v_t_z = atof(it.second.c_str());
		//}
		//else if (it.first == "x_control") {
		//	this->x_c_0 = atof(it.second.c_str());
		//}
		//else if (it.first == "y_control") {
		//	this->y_c_0 = atof(it.second.c_str());
		//}
		//else if (it.first == "z_control") {
		//	this->z_c_0 = atof(it.second.c_str());
		//}
		//else if (it.first == "v_control_x") {
		//	this->v_c_x = atof(it.second.c_str());
		//}
		//else if (it.first == "v_control_y") {
		//	this->v_c_y = atof(it.second.c_str());
		//}
		//else if (it.first == "v_control_z") {
		//	this->v_c_z = atof(it.second.c_str());
		//}
		//else if (it.first == "distance_of_stop_nav") {
		//	this->distance_of_stop_nav = atof(it.second.c_str());
		//}
		if (it.first == "guidance_movement_filepath") {
			guidance_station_movement_.Load(it.second.c_str(), tj);
			//this->LoadGuidanceFile(it.second.c_str());
		}
		else if (it.first == "target_movement_filepath") {
			target_movement_.Load(it.second.c_str(), tj);
			//this->LoadTargetFile(it.second.c_str());
		}
		else if (it.first == "d_epsilon0_h") {
			this->d_epsilon0_h = atof(it.second.c_str())/RAD;
		}
		else if (it.first == "d_epsilon0_v") {
			this->d_epsilon0_v = atof(it.second.c_str())/RAD;
		}
		else {
			printf("Qianzhi_Navigation_ControlModel, δ֪������:%s\n", it.first.c_str());
			prompt_exit(1);
		}
	});
}



void BestCL_CD_ControlModel::control_equations(double t, double dt, double y[], double* delta_y, double* delta_z, double* alpha, double*beta, double* gamma_v, three_degree_trajectory* tj) {
	double V = y[three_degree_trajectory::V_Index];

	//������ƹ��ɣ� ���ƶ�ƫ��
	double t_interp = t - t_start_; //��ֵʱҪ��ȥ���ʱ��
									//*gamma_v = 0;
	//��������
	double altitude = y[three_degree_trajectory::Y_Index]; //��ǰ�����߶�
	double sonic = g_atmosphere_model.GetSonic(altitude);
	//double rho = g_atmosphere_model.GetRHO(altitude);
	double mach = V / sonic;

	*gamma_v = 0;
	*beta = 0;
	*alpha = tj->_aero_model->GetMaxCLCD_Alpha(t, mach, *beta);
	//*alpha = GetMaxCLCD_Alpha_Rad(tj->aero_data, mach, *beta);

	//ƽ���ϵʽ�����ݵ�ǰ�Ĺ��ǺͲ໬�ǣ����Ӧ��ƽ���ƫ��
	*delta_y = tj->_aero_model->GetBalanceDeltaY(t, mach, *alpha, *beta);
	*delta_z = tj->_aero_model->GetBalanceDeltaZ(t, mach, *alpha, *beta);
	//*delta_y = GetBalanceDeltaY_Rad(tj->aero_data, mach, *alpha, *beta);
	//*delta_z = GetBalanceDeltaZ_Rad(tj->aero_data, mach, *alpha, *beta);
	//fprintf(tj->f_zhixin, "%15.10f %15.10f %15.10f\n", t, y[three_degree_trajectory::Y_Index], y[three_degree_trajectory::Z_Index]);
};

void AlphaBetaGammaV_ControlModel::control_equations(double t, double dt, double y[], double* delta_y, double* delta_z, double* alpha, double*beta, double* gamma_v, three_degree_trajectory* tj) {
	double V = y[three_degree_trajectory::V_Index];

	//������ƹ��ɣ� ���ƶ�ƫ��
	double t_interp = t - t_start_; //��ֵʱҪ��ȥ���ʱ��
	//*gamma_v = 0;
	if (gammav_t_array.size() != 0 && gammav_array.size() != 0)
		*gamma_v = interp11(gammav_array, gammav_t_array, t_interp);
	else *gamma_v = 0;
	if (alpha_t_array.size() != 0 && alpha_array.size() != 0)
		*alpha = interp11(alpha_array, alpha_t_array, t_interp);
	else *alpha = 0;
	if (beta_t_array.size() != 0 && beta_array.size() != 0)
		*beta = interp11(beta_array, beta_t_array, t_interp);
	else *beta = 0;

	//��������
	double altitude = y[three_degree_trajectory::Y_Index]; //��ǰ�����߶�
	double sonic = g_atmosphere_model.GetSonic(altitude);
	//double rho = g_atmosphere_model.GetRHO(altitude);
	double mach = V / sonic;

	//ƽ���ϵʽ�����ݵ�ǰ�Ĺ��ǺͲ໬�ǣ����Ӧ��ƽ���ƫ��
	*delta_y = tj->_aero_model->GetBalanceDeltaY(t, mach, *alpha, *beta);
	*delta_z = tj->_aero_model->GetBalanceDeltaZ(t, mach, *alpha, *beta);
	//*delta_y = GetBalanceDeltaY_Rad(tj->aero_data, mach, *alpha, *beta);
	//*delta_z = GetBalanceDeltaZ_Rad(tj->aero_data, mach, *alpha, *beta);
	//fprintf(tj->f_zhixin, "%15.10f %15.10f %15.10f\n", t, y[three_degree_trajectory::Y_Index], y[three_degree_trajectory::Z_Index]);
};

void VarTheta_Psi_ControlModel::control_equations(double t, double dt, double y[], double* delta_y, double* delta_z, double* alpha, double*beta, double* gamma_v, three_degree_trajectory* tj) {
	double V = y[three_degree_trajectory::V_Index];
	double theta = y[three_degree_trajectory::Theta_Index]; //theta�ᱻֱ���޸ģ���ɵ�Ч���ǣ�ʵ���ϲ����������
	double psi_v = y[three_degree_trajectory::Psi_V_Index];

	//������ƹ��ɣ� 
	double t_interp = t - t_start_; //��ֵʱҪ��ȥ���ʱ��
	*gamma_v = 0;

	double vartheta = VarTheta(t);
	double psi = Psi(t);

	//��������
	double altitude = y[three_degree_trajectory::Y_Index]; //��ǰ�����߶�
	double sonic = g_atmosphere_model.GetSonic(altitude);
	//double rho = g_atmosphere_model.GetRHO(altitude);
	double mach = V / sonic;

	*beta = asin(cos(theta) * sin(psi - psi_v));
	*alpha = acos(
		( cos(vartheta)*cos(theta)*cos(psi - psi_v) + sin(vartheta)*sin(theta)  )
		/
		cos(*beta)
		);

	//ƽ���ϵʽ�����ݵ�ǰ�Ĺ��ǺͲ໬�ǣ����Ӧ��ƽ���ƫ��
	*delta_y = tj->_aero_model->GetBalanceDeltaY(t, mach, *alpha, *beta);
	*delta_z = tj->_aero_model->GetBalanceDeltaZ(t, mach, *alpha, *beta);

	//fprintf(tj->f_zhixin, "%15.10f %15.10f %15.10f\n", t, y[three_degree_trajectory::Y_Index], y[three_degree_trajectory::Z_Index]);
};

//Ҫ��50����أ����������߹��ɣ� y = 12000-a*t*t, a = 12000/2500��������ɴﲻ��
//����ֱ�߹��ɣ� 60�����, y = 12000-a*t, a=12000/60 
//inline double H(double t){
//	double a = 12000 / (double)60;
//	return 12000 - a*t;
//}

//��Ӧƽ�⹥���µ�����
// ���룺 �ο����ǵ� ��mach����ѹ���ο��໬��
inline double calc_y_balance(double t, AerodynamicModel* aero_model, double mach, double q, double S, double beta, double alpha) {
	double cy_balance = aero_model->GetCYBalanceByAlpha(t, mach, alpha, beta);
	return cy_balance*q*S;
}

//��Ӧƽ�⹥���µĹ���
// ���룺 �������ο����ǵ� ��
inline double calc_ny3(double t, AerodynamicModel* aero_model, double mach, double q, double S, double beta, double alpha, double P, double m) {
	double y_balance = calc_y_balance(t, aero_model, mach, q, S, beta, alpha);
	return (P*sin(alpha) + y_balance) / (m*G);
}

//�󣨵�ǰ�����£����ضԹ��ǵĵ��� ���򻯹�ʽ���� ���������С���Ǽ���򻯣���P*sin(alpha)�򻯳���P*alpha �������������ѧ�� P67
double calc_ny3_alpha_simple(double t, AerodynamicModel* aero_model, double mach, double q, double S, double beta, double alpha, double P, double m) {
	double y_balance = calc_y_balance(t, aero_model, mach, q, S, beta, alpha);
	return (P + y_balance / alpha) / (m*G);
}

//�󣨵�ǰ�����£����ضԹ��ǵĵ��� ���Ǽ򻯹�ʽ��
double calc_ny3_alpha(double t, AerodynamicModel* aero_model, double mach, double q, double S, double beta, double alpha, double P, double m) {
	double y_balance = calc_y_balance(t, aero_model, mach, q, S, beta, alpha);
	return (P*sin(alpha) + y_balance) / (m*G) / alpha;
}

//���ݷ�����أ�ny3�����Ӧ��ƽ�⹥�ǣ������������ѧ�� P67 , 
inline double calc_alphaB_by_ny3(double t, AerodynamicModel* aero_model, double mach, double q, double S, double beta, double P, double m, double ny3_required, double previous_alpha) {

	// �����ƽ�⹥���µĹ���
	//double alpha0 = 0;
	//double delta_z0 = GetBalanceDeltaZ_Rad(tj->aero_data, mach, alpha0, *beta);
	//double CY0_Balance = GetCY_Rad(tj->aero_data, mach, alpha0, *beta, 0, *delta_y, delta_z0);
	//double Y0_Balance = CY0_Balance*q*S;
	double Y0_Balance = calc_y_balance(t, aero_model, mach, q, S, beta, 0);
	double ny3_0 = Y0_Balance / (m*G);
	if (abs(ny3_required - ny3_0) < 0.00000000001) //��Ӧ0���ǵĹ���
		return 0;

	//����ض�ƽ�⹥�ǵĵ����� ��Ե�ǰP��mach,beta,delta_y�������£� ���������� ��һ��״̬�ģ����Ƕ�Ӧ�ģ�����/����
	//double previous_alpha = tj->_states[three_degree_trajectory::Alpha_Index];
	if (abs(previous_alpha) < 0.000001)//��һ��״̬�Ĺ���Ϊ0���޷�������ضԹ��ǵĵ���������������ټ���
		previous_alpha = 5 / RAD; //
								  //double delta_z_balance = GetBalanceDeltaZ_Rad(tj->aero_data, mach, previous_alpha, *beta);
								  //double CY_Balance = GetCY_Rad(tj->aero_data, mach, previous_alpha, *beta, 0, 0, delta_z_balance);
								  //double Y_Balance = CY_Balance*q*S;
								  //double ny3_balance_alpha = (P + Y_Balance / previous_alpha) / (m*G);
	//double ny3_balance_alpha = calc_ny3_alpha_simple(aero_data, mach, q, S, beta, previous_alpha, P, m);
	double ny3_balance_alpha = calc_ny3_alpha(t, aero_model, mach, q, S, beta, previous_alpha, P, m);
	//���ݶ�Ӧ�ĵ�������һ����Ŀ��ֵ�ȽϽӽ��Ĺ���
	double alpha = (ny3_required - ny3_0) / ny3_balance_alpha;
	int max_count = 3; //����������
	int i = 0;
	//�����޴ε����ƽ�������ض�Ӧ��ƽ�⹥�ǣ�������Բ�ȡ�ķ����Ƕ��ַ������߲�ֵ������Ҫ�ȹ��쵱ǰ������ny3��ƽ�⹥�ǵ����飩��
	while (abs(alpha - previous_alpha) > (0.125 / RAD)) { //����������õ�alpha��previous_alpha�ܽӽ�(С��0.125��)��˵������previous_alpha�Թ��صĵ�������alpha�Ǻ���ģ�������ֹ
		//ny3_balance_alpha = calc_ny3_alpha_simple(aero_data, mach, q, S, beta, alpha, P, m);
		ny3_balance_alpha = calc_ny3_alpha(t, aero_model, mach, q, S, beta, alpha, P, m);
		previous_alpha = alpha;
		alpha = (ny3_required - ny3_0) / ny3_balance_alpha;
		i++;
		if (i >= max_count)
			break;
	}


	//���ӹ����޷��� �����޷���������ش����ں����ĵ�������ģ�鱻���֣������ȡ������
	//if (alpha > (aero_model->GetMaxAlpha() / RAD))
	//	alpha = aero_model->GetMaxAlpha() / RAD;
	//if (alpha < (aero_model->GetMinAlpha() / RAD))
	//	alpha = aero_model->GetMinAlpha() / RAD;

	return alpha;
}

//���ݷ�����أ�ny3�����Ӧ��ƽ�⹥�ǣ�����sin(alpha)�򻯳�alpha, ny3 = ( P*sin(alpha) + Y )/G 
//���ö��ַ�
inline double calc_alphaB_by_ny3_bisection(double t, AerodynamicModel* aero_model, double mach, double q, double S, double beta, double P, double m, double ny3_required, double previous_alpha) {
	//double alpha_max = 30 / RAD;
	//double alpha_min = -30 / RAD;
	double alpha_max = aero_model->GetMaxAlpha() / RAD;
	double alpha_min = aero_model->GetMinAlpha() / RAD;
	double ny3_max = calc_ny3(t, aero_model, mach, q, S, beta, alpha_max, P, m);
	double ny3_min = calc_ny3(t, aero_model, mach, q, S, beta, alpha_min, P, m);
	if (ny3_max < ny3_required)
		return alpha_max;  //�������㷶Χ��60�ȹ����Ѿ����Ǻ���Ŀ��ƹ����ˣ�
	if (ny3_min > ny3_required)
		return alpha_min;  //�������㷶Χ��60�ȹ����Ѿ����Ǻ���Ŀ��ƹ����ˣ�


	double ny3_middle;
	double alpha_middle;
	do {
		alpha_middle = (alpha_max + alpha_min) / 2;
		ny3_middle = calc_ny3(t, aero_model, mach, q, S, beta, alpha_middle, P, m);
		if (ny3_required > ny3_middle) {
			alpha_min = alpha_middle;
			ny3_min = ny3_middle;
			continue;
		}
		else if (ny3_required < ny3_middle) {
			alpha_max = alpha_middle;
			ny3_max = ny3_middle;
			continue;
		}
		else return alpha_middle;
	} while (abs(ny3_middle - ny3_required) > abs(ny3_required / 100));  //�������С��1%��������Ϊ��Ϊ���󹥽Ƿ�������
	return alpha_middle;
}

//����dtheta_dt���Ӧ��ƽ�⹥��
double calc_alphaB_by_dtheta_dt(double t, AerodynamicModel* aero_model, double mach, double q, double S, double beta, double gamma_v, double P, double m, double V, double dtheta_dt, double dpsiv_dt, double theta, double previous_alpha) {
	//���÷������ny2������ƽ����ny2�ļ��㹫ʽ��  ����Ӧ��ת��ͨ�ù�ʽ����������������Ļ����ny2
	double ny2_required = (V / G)*(dtheta_dt)+cos(theta);
	double nz2_required = -(V / G)*cos(theta)*(dpsiv_dt);
	//��������ƽ�棨Yv=0ʱ���� ny3=ny2�� ����Ӧ��ͨ��L(Yv)����ny3
	double ny3_required = ny2_required;
	return calc_alphaB_by_ny3(t, aero_model, mach, q, S, beta, P, m, ny3_required, previous_alpha);
	//return calc_alphaB_by_ny3_bisection(t, aero_model, mach, q, S, beta, P, m, ny3_required, previous_alpha);
}



//��Ӧƽ�⹥���µ�����
// ���룺 �ο����ǵ� ��mach����ѹ���ο��໬��
inline double calc_z_balance(double t, AerodynamicModel* aero_model, double mach, double q, double S, double beta, double alpha) {
	double cz_balance = aero_model->GetCZBalanceByBeta(t, mach, alpha, beta);
	return cz_balance*q*S;
}

//�󣨵�ǰ�����£�����nz3�Բ໬�ǵĵ��� ���Ǽ򻯹�ʽ��
double calc_nz3_beta(double t, AerodynamicModel* aero_model, double mach, double q, double S, double beta, double alpha, double P, double m) {
	double z_balance = calc_z_balance(t, aero_model, mach, q, S, beta, alpha);
	return (-P*cos(alpha)*sin(beta) + z_balance) / (m*G) / beta;
}

//��Ӧƽ��໬���µĹ���
// ���룺 �������ο��໬�ǵ� ��
inline double calc_nz3(double t, AerodynamicModel* aero_model, double mach, double q, double S, double beta, double alpha, double P, double m) {
	//double y_balance = calc_y_balance(t, aero_model, mach, q, S, beta, alpha);
	double z_balance = calc_z_balance(t, aero_model, mach, q, S, beta, alpha);
	return (-P*cos(alpha)*sin(beta) + z_balance) / (m*G);
}

//���ݷ�����أ�ny3�����Ӧ��ƽ��໬�ǣ�������ҵ��ѧ
//���ö��ַ�
inline double calc_betaB_by_nz3_bisection(double t, AerodynamicModel* aero_model, double mach, double q, double S, double alpha, double P, double m, double nz3_required, double previous_beta) {
	//double beta_max = -20 / RAD;
	//double beta_min = 20 / RAD;
	double beta_max = aero_model->GetMinBeta() / RAD;
	double beta_min = aero_model->GetMaxBeta() / RAD;
	double nz3_max = calc_nz3(t, aero_model, mach, q, S, beta_max, alpha, P, m);
	double nz3_min = calc_nz3(t, aero_model, mach, q, S, beta_min, alpha, P, m);
	if (nz3_max < nz3_required)
		return beta_max;  //�������㷶Χ��60�ȹ����Ѿ����Ǻ���Ŀ��ƹ����ˣ�
	if (nz3_min > nz3_required)
		return beta_min;  //�������㷶Χ��60�ȹ����Ѿ����Ǻ���Ŀ��ƹ����ˣ�

	double nz3_middle;
	double beta_middle;
	do {
		beta_middle = (beta_max + beta_min) / 2;
		nz3_middle = calc_nz3(t, aero_model, mach, q, S, beta_middle, alpha, P, m);
		if (nz3_required > nz3_middle) {
			beta_min = beta_middle;
			nz3_min = nz3_middle;
			continue;
		}
		else if (nz3_required < nz3_middle) {
			beta_max = beta_middle;
			nz3_max = nz3_middle;
			continue;
		}
		else return beta_middle;
	} while (abs(nz3_middle - nz3_required) > abs(nz3_required / 100));  //�������С��1%��������Ϊ��Ϊ����໬�Ƿ�������
	return beta_middle;
}

//���ݷ�����أ�nz3�����Ӧ��ƽ��໬�ǣ������������ѧ 
inline double calc_betaB_by_nz3(double t, AerodynamicModel* aero_model, double mach, double q, double S, double alpha, double P, double m, double nz3_required, double previous_beta) {

	// ����Ȳ໬���µĹ���
	double Z0_Balance = calc_z_balance(t, aero_model, mach, q, S, 0, alpha);
	double nz3_0 = Z0_Balance / (m*G);
	if (abs(nz3_required - nz3_0) < 0.0000000001)
		return 0;

	//����ض�ƽ��໬�ǽǵĵ����� ��Ե�ǰP��mach,beta,delta_y�������£� ���������� ��һ��״̬�ģ��໬�Ƕ�Ӧ�ģ�����/�໬��
	if (abs(previous_beta) < 0.000001)//��һ��״̬�Ĺ���Ϊ0���޷�������ضԲ໬�ǵĵ���������޸�Ϊһ������ֵ��5�ȣ��ټ���
		previous_beta = 5 / RAD; //
	double nz3_balance_beta = calc_nz3_beta(t, aero_model, mach, q, S, previous_beta, alpha, P, m);
	//���ݶ�Ӧ�ĵ�������һ����Ŀ��ֵ�ȽϽӽ��Ĳ໬��
	double beta = (nz3_required - nz3_0) / nz3_balance_beta;
	int max_count = 3; //����������
	int i = 0;
	while (abs(beta - previous_beta) > (0.125 / RAD)) { //����������õ�beta��previous_beta�ܽӽ�(С��0.125��)��˵������previous_beta�Թ��صĵ�������beta�Ǻ���ģ�������ֹ
		nz3_balance_beta = calc_nz3_beta(t, aero_model, mach, q, S, beta, alpha, P, m);
		previous_beta = beta;
		beta = (nz3_required - nz3_0) / nz3_balance_beta;
		i++;
		if (i >= max_count)
			break;
	}

	//���Ӳ໬���޷���ȡ��ԭ��ͬ�����޷���
	//if (beta > (aero_model->GetMaxBeta()/RAD) )
	//	beta = aero_model->GetMaxBeta()/RAD;
	//if (beta < (aero_model->GetMinBeta()/RAD))
	//	beta = aero_model->GetMinBeta()/RAD;

	return beta;
}

//����dpsi_v_dt���Ӧ��ƽ��໬��
double calc_betaB_by_dpsi_v_dt(double t, AerodynamicModel* aero_model, double mach, double q, double S, double alpha, double gamma_v, double P, double m, double V, double dtheta_dt, double dpsiv_dt, double theta, double previous_beta) {
	//���÷������ny2������ƽ����ny2�ļ��㹫ʽ��  ����Ӧ��ת��ͨ�ù�ʽ����������������Ļ����ny2
	double ny2_required = (V / G)*(dtheta_dt)+cos(theta);
	double nz2_required = -(V / G)*cos(theta)*(dpsiv_dt);
	//��������ƽ�棨Yv=0ʱ���� ny3=ny2�� ����Ӧ��ͨ��L(Yv)����ny3
	double ny3_required = ny2_required;
	double nz3_required = nz2_required;
	return calc_betaB_by_nz3(t, aero_model, mach, q, S, alpha, P, m, nz3_required, previous_beta);
	//return calc_betaB_by_nz3_bisection(t, aero_model, mach, q, S, alpha, P, m, nz3_required, previous_beta);
}

double ThetaPsiV_ControlModel::Theta(double t) {
	if (theta_t_array.size() != 0 && theta_array.size() != 0)
		return interp11(theta_array, theta_t_array, t - t_start_);
	else return 0;
}
double ThetaPsiV_ControlModel::Psi_V(double t) {
	if (psi_v_t_array.size() != 0 && psi_v_array.size() != 0)
		return interp11(psi_v_array, psi_v_t_array, t - t_start_);
	else return 0;
}

double VarTheta_Psi_ControlModel::VarTheta(double t) {
	if (vartheta_t_array.size() != 0 && vartheta_array.size() != 0)
		return interp11(vartheta_array, vartheta_t_array, t - t_start_);
	else return 0;
}
double VarTheta_Psi_ControlModel::Psi(double t) {
	if (psi_t_array.size() != 0 && psi_array.size() != 0)
		return interp11(psi_array, psi_t_array, t - t_start_);
	else return 0;
}

double Ny2Nz2_ControlModel::Ny2(double t) {
	if (ny2_t_array.size() != 0 && ny2_array.size() != 0)
		return interp11(ny2_array, ny2_t_array, t - t_start_);
	else return 0;
}
double Ny2Nz2_ControlModel::Nz2(double t) {
	if (nz2_t_array.size() != 0 && nz2_array.size() != 0)
		return interp11(nz2_array, nz2_t_array, t - t_start_);
	else return 0;
}

//double Theta_ControlModel::Psiv(double t) {
//	return interp11(theta_array, theta_t_array, t - t_start_);
//}



//������ǿ���ģ�� 
void ThetaPsiV_ControlModel::control_equations(double t, double dt, double y[], double* delta_y, double* delta_z, double* alpha, double*beta, double* gamma_v, three_degree_trajectory* tj) {
	double const& V = y[three_degree_trajectory::V_Index];
	double& theta = y[three_degree_trajectory::Theta_Index]; //theta�ᱻֱ���޸ģ���ɵ�Ч���ǣ�ʵ���ϲ����������
	double& psi_v = y[three_degree_trajectory::Psi_V_Index];
	double const& m = y[three_degree_trajectory::Mass_Index];

	//������ƹ��ɣ� �ٶ���б�ǿ���Ϊ0
	*gamma_v = 0;
	//*beta = 0;
	//double t_interp = t - t_start_; //��ֵʱҪ��ȥ���ʱ��

	//��������
	double altitude = y[three_degree_trajectory::Y_Index]; //��ǰ�����߶�
	double sonic = g_atmosphere_model.GetSonic(altitude);
	double rho = g_atmosphere_model.GetRHO(altitude);
	double mach = V / sonic;
	//double S = GetS(tj->aero_data);
	double S = tj->_aero_model->GetS();
	double q = 0.5 * rho * V * V;
	double P = tj->_thrust_model->get_current_thrust(t);

	//���ݵ�����Ǳ仯������dtheta_dt
	theta = Theta(t);
	double theta_next = Theta(t + dt);
	double dtheta_dt = (theta_next - theta) / dt;


	//���ݵ���ƫ�Ǳ仯������dpsiv_dt
	psi_v = Psi_V(t);
	double psi_v_next = Psi_V(t + dt);
	double dpsi_v_dt = (psi_v_next - psi_v) / dt;

	double previous_alpha = tj->_alpha;
	double previous_beta = tj->_beta;
	*alpha = calc_alphaB_by_dtheta_dt(t, tj->_aero_model, mach, q, S, previous_beta, *gamma_v, P, m, V, dtheta_dt, dpsi_v_dt, theta, previous_alpha);//)
	*beta = calc_betaB_by_dpsi_v_dt(t, tj->_aero_model, mach, q, S, *alpha, *gamma_v, P, m, V, dtheta_dt, dpsi_v_dt, theta, previous_beta);
	//*alpha = calc_alphaB_by_dtheta_dt(tj->aero_data, mach, q, S, previous_beta, *gamma_v, P, m, V, dtheta_dt, dpsi_v_dt, theta, previous_alpha);//)
	//*beta = calc_betaB_by_dpsi_v_dt(tj->aero_data, mach, q, S, *alpha, *gamma_v, P, m, V, dtheta_dt, dpsi_v_dt, theta, previous_beta);

	//ƽ���ϵʽ�����ݵ�ǰ�Ĺ��ǺͲ໬�ǣ����Ӧ��ƽ���ƫ��
	*delta_y = tj->_aero_model->GetBalanceDeltaY(t, mach, *alpha, *beta);
	*delta_z = tj->_aero_model->GetBalanceDeltaZ(t, mach, *alpha, *beta);
	//*delta_y = GetBalanceDeltaY_Rad(tj->aero_data, mach, *alpha, *beta);
	//*delta_z = GetBalanceDeltaZ_Rad(tj->aero_data, mach, *alpha, *beta);

	//fprintf(tj->f_zhixin, "%15.10f %15.10f %15.10f\n", t, y[three_degree_trajectory::Y_Index], y[three_degree_trajectory::Z_Index]);
};

void Ny2Nz2_ControlModel::control_equations(double t, double dt, double y[], double* delta_y, double* delta_z, double* alpha, double*beta, double* gamma_v, three_degree_trajectory* tj) {
	double const& V = y[three_degree_trajectory::V_Index];
	double& theta = y[three_degree_trajectory::Theta_Index]; //theta�ᱻֱ���޸ģ���ɵ�Ч���ǣ�ʵ���ϲ����������
	double& psi_v = y[three_degree_trajectory::Psi_V_Index];
	double const& m = y[three_degree_trajectory::Mass_Index];

	//������ƹ��ɣ� �ٶ���б�ǿ���Ϊ0
	*gamma_v = 0;
	//*beta = 0;
	//double t_interp = t - t_start_; //��ֵʱҪ��ȥ���ʱ��

	//��������
	double altitude = y[three_degree_trajectory::Y_Index]; //��ǰ�����߶�
	double sonic = g_atmosphere_model.GetSonic(altitude);
	double rho = g_atmosphere_model.GetRHO(altitude);
	double mach = V / sonic;
	//double S = GetS(tj->aero_data);
	double S = tj->_aero_model->GetS();
	double q = 0.5 * rho * V * V;
	double P = tj->_thrust_model->get_current_thrust(t);

	// ny2 �� nz2
	double ny2 = Ny2(t);
	double nz2 = Nz2(t);

	//��������ƽ�棨Yv=0ʱ���� ny3=ny2�� ����Ӧ��ͨ��L(Yv)����ny3��nz3
	double ny3_required = ny2;
	double nz3_required = nz2;


	double previous_alpha = tj->_alpha;
	double previous_beta = tj->_beta;
	*alpha =  calc_alphaB_by_ny3(t, tj->_aero_model, mach, q, S, previous_beta, P, m, ny3_required, previous_alpha);
	*beta = calc_betaB_by_nz3(t, tj->_aero_model, mach, q, S, *alpha, P, m, nz3_required, previous_beta);
	//*alpha = calc_alphaB_by_dtheta_dt(t, tj->_aero_model, mach, q, S, previous_beta, *gamma_v, P, m, V, dtheta_dt, dpsi_v_dt, theta, previous_alpha);//)
	//*beta = calc_betaB_by_dpsi_v_dt(t, tj->_aero_model, mach, q, S, *alpha, *gamma_v, P, m, V, dtheta_dt, dpsi_v_dt, theta, previous_beta);

	//ƽ���ϵʽ�����ݵ�ǰ�Ĺ��ǺͲ໬�ǣ����Ӧ��ƽ���ƫ��
	*delta_y = tj->_aero_model->GetBalanceDeltaY(t, mach, *alpha, *beta);
	*delta_z = tj->_aero_model->GetBalanceDeltaZ(t, mach, *alpha, *beta);

	//fprintf(tj->f_zhixin, "%15.10f %15.10f %15.10f\n", t, y[three_degree_trajectory::Y_Index], y[three_degree_trajectory::Z_Index]);
};


double YZ_ControlModel::H(double t) {
	return interp11(y_array, y_t_array, t - t_start_);
}

double YZ_ControlModel::Z(double t) {
	if (z_array.size() != 0 && z_t_array.size() != 0)
		return interp11(z_array, z_t_array, t - t_start_);
	else return 0;
}


//�������Ŀ���ģ�� 
void YZ_ControlModel::control_equations(double t, double dt, double y[], double* delta_y, double* delta_z, double* alpha, double*beta, double* gamma_v, three_degree_trajectory* tj) {
	double const& V = y[three_degree_trajectory::V_Index];
	double& theta = y[three_degree_trajectory::Theta_Index]; //theta�ᱻֱ���޸ģ���ɵ�Ч���ǣ�ʵ���ϲ����������
	double& psi_v = y[three_degree_trajectory::Psi_V_Index]; //psi_v�ᱻֱ���޸ģ���ɵ�Ч���ǣ�ʵ���ϲ����������
	double const& m = y[three_degree_trajectory::Mass_Index];

	//������ƹ��ɣ� �����������ģ��ٶ���б�ǺͲ໬�Ƕ�����Ϊ0
	*gamma_v = 0;
	*beta = 0;

	//�����������ĺͲ�������
	y[three_degree_trajectory::Y_Index] = H(t);
	y[three_degree_trajectory::Z_Index] = Z(t);


	//��������
	double altitude = y[three_degree_trajectory::Y_Index]; //��ǰ�����߶�
	double sonic = g_atmosphere_model.GetSonic(altitude);
	double rho = g_atmosphere_model.GetRHO(altitude);
	double mach = V / sonic;
	//double S = GetS(tj->aero_data);
	double S = tj->_aero_model->GetS();
	double q = 0.5 * rho * V * V;
	double P = tj->_thrust_model->get_current_thrust(t);

	//���ݸ߶ȱ仯������dtheta_dt
	double dh_dt = (H(t + dt) - H(t)) / dt;
	if (fabs(dh_dt) > fabs(V)) {
		printf("����������ı仯���ɣ� �������ı仯�ٶ�(%f) ���� ������ǰ�ٶ�(%f)\n ", dh_dt, V);
		prompt_exit(1);
	}
	theta = asin(dh_dt / V); //�����������
	double dh_dt_next = (H(t + 2 * dt) - H(t + dt)) / dt;
	//�����������V��������V_Next����Ϊ�����С�Ĳ���������0.01�£����ڼ������صĵ��������ٶȼ�ʮ��/�룬������ٸ����ĵ���������ٶ���Ա仯��Լ��0.1/s���ң���V�����ԼΪǧ��֮һ
	double theta_next = asin(dh_dt_next / V);
	double dtheta_dt = (theta_next - theta) / dt;


	//���ݲ������ı仯������dpsiv_dt
	double dz_dt = (Z(t + dt) - Z(t)) / dt;
	if (fabs(dz_dt) > fabs(V*cos(theta))) {
		printf("����������ı仯���ɣ� �������ı仯�ٶ�(%f) >  ������ǰ�ٶ�(%f) * cos(�������(%f)) \n ", dz_dt, V, theta*RAD);
		prompt_exit(1);
	}
	psi_v = asin( dz_dt / (-V * cos(theta) ) ); //��������ƫ��
	double dz_dt_next = (Z(t + 2 * dt) - Z(t + dt)) / dt;
	//�����������V��������V_Next����Ϊ�����С�Ĳ���������0.01�£����ڼ������صĵ��������ٶȼ�ʮ��/�룬������ٸ����ĵ���������ٶ���Ա仯��Լ��0.1/s���ң���V�����ԼΪǧ��֮һ
	double psi_v_next = asin(dz_dt_next / (-V* cos(theta_next)));
	double dpsi_v_dt = (psi_v_next - psi_v) / dt;
	if (fabs(dpsi_v_dt) < 1e-7) //���˵�һЩ̫С��������Ϊ��������ѧ������
		dpsi_v_dt = 0;
	else {
		int i = 0;
	}




	double previous_alpha = tj->_alpha;
	double previous_beta = tj->_beta;
	*alpha = calc_alphaB_by_dtheta_dt(t, tj->_aero_model, mach, q, S, previous_beta, *gamma_v, P, m, V, dtheta_dt, dpsi_v_dt, theta, previous_alpha);//)
	*beta = calc_betaB_by_dpsi_v_dt(t, tj->_aero_model, mach, q, S, *alpha, *gamma_v, P, m, V, dtheta_dt, dpsi_v_dt, theta, previous_beta);

	//ƽ���ϵʽ�����ݵ�ǰ�Ĺ��ǺͲ໬�ǣ����Ӧ��ƽ���ƫ��
	*delta_y = tj->_aero_model->GetBalanceDeltaY(t, mach, *alpha, *beta);
	*delta_z = tj->_aero_model->GetBalanceDeltaZ(t, mach, *alpha, *beta);

	//fprintf(tj->f_zhixin, "%15.10f %15.10f %15.10f\n", t, y[three_degree_trajectory::Y_Index], y[three_degree_trajectory::Z_Index]);
};


double XYZ_ControlModel::H(double X_input) {
	return interp11(y_array, x_array, X_input);
}

double XYZ_ControlModel::Z(double X_input) {
	if (z_array.size() != 0 && x_array.size() != 0)
		return interp11(z_array, x_array, X_input);
	else return 0;
}


//XYZ���Ŀ���ģ�� 
void XYZ_ControlModel::control_equations(double t, double dt, double y[], double* delta_y, double* delta_z, double* alpha, double*beta, double* gamma_v, three_degree_trajectory* tj) {
	double const& V = y[three_degree_trajectory::V_Index];
	double& theta = y[three_degree_trajectory::Theta_Index]; //theta�ᱻֱ���޸ģ���ɵ�Ч���ǣ�ʵ���ϲ����������
	double& psi_v = y[three_degree_trajectory::Psi_V_Index]; //psi_v�ᱻֱ���޸ģ���ɵ�Ч���ǣ�ʵ���ϲ����������
	double const& m = y[three_degree_trajectory::Mass_Index];

	double x_input = y[three_degree_trajectory::X_Index];
	double dx = V*cos(theta)*cos(psi_v)*dt;
	double x_next1 = x_input + dx;
	double x_next2 = x_next1 + dx;
	//������ƹ��ɣ� �����������ģ��ٶ���б�ǺͲ໬�Ƕ�����Ϊ0
	*gamma_v = 0;
	*beta = 0;

	//�����������ĺͲ�������
	y[three_degree_trajectory::Y_Index] = H(x_input);
	y[three_degree_trajectory::Z_Index] = Z(x_input);


	//��������
	double altitude = y[three_degree_trajectory::Y_Index]; //��ǰ�����߶�
	double sonic = g_atmosphere_model.GetSonic(altitude);
	double rho = g_atmosphere_model.GetRHO(altitude);
	double mach = V / sonic;
	//double S = GetS(tj->aero_data);
	double S = tj->_aero_model->GetS();
	double q = 0.5 * rho * V * V;
	double P = tj->_thrust_model->get_current_thrust(t);

	//���ݸ߶ȱ仯������dtheta_dt
	double dh_dx = (H(x_next1) - H(x_input)) / dx;
	if (fabs(dh_dx) > fabs(V)) {
		printf("����������ı仯���ɣ� �������ı仯�ٶ�(%f) ���� ������ǰ�ٶ�(%f)\n ", dh_dx, V);
		prompt_exit(1);
	}
	theta = atan(dh_dx); //�����������
	double dh_dx_next = (H(x_next2) - H(x_next1)) / dx;
	//�����������V��������V_Next����Ϊ�����С�Ĳ���������0.01�£����ڼ������صĵ��������ٶȼ�ʮ��/�룬������ٸ����ĵ���������ٶ���Ա仯��Լ��0.1/s���ң���V�����ԼΪǧ��֮һ
	double theta_next = atan(dh_dx_next);
	double dtheta_dt = (theta_next - theta) / dt;


	//���ݲ������ı仯������dpsiv_dt
	double dz_dx = (Z(x_next1) - Z(x_input)) / dx;
	if (fabs(dz_dx) > fabs(V*cos(theta))) {
		printf("����������ı仯���ɣ� �������ı仯�ٶ�(%f) >  ������ǰ�ٶ�(%f) * cos(�������(%f)) \n ", dz_dx, V, theta*RAD);
		prompt_exit(1);
	}
	psi_v = atan(dz_dx); //��������ƫ��
	double dz_dx_next = (Z(x_next2) - Z(x_next1)) / dx;
	//�����������V��������V_Next����Ϊ�����С�Ĳ���������0.01�£����ڼ������صĵ��������ٶȼ�ʮ��/�룬������ٸ����ĵ���������ٶ���Ա仯��Լ��0.1/s���ң���V�����ԼΪǧ��֮һ
	double psi_v_next = atan(dz_dx_next);
	double dpsi_v_dt = (psi_v_next - psi_v) / dt;
	if (fabs(dpsi_v_dt) < 1e-7) //���˵�һЩ̫С��������Ϊ��������ѧ������
		dpsi_v_dt = 0;
	else {
		int i = 0;
	}




	double previous_alpha = tj->_alpha;
	double previous_beta = tj->_beta;
	*alpha = calc_alphaB_by_dtheta_dt(t, tj->_aero_model, mach, q, S, previous_beta, *gamma_v, P, m, V, dtheta_dt, dpsi_v_dt, theta, previous_alpha);//)
	*beta = calc_betaB_by_dpsi_v_dt(t, tj->_aero_model, mach, q, S, *alpha, *gamma_v, P, m, V, dtheta_dt, dpsi_v_dt, theta, previous_beta);

	//ƽ���ϵʽ�����ݵ�ǰ�Ĺ��ǺͲ໬�ǣ����Ӧ��ƽ���ƫ��
	*delta_y = tj->_aero_model->GetBalanceDeltaY(t, mach, *alpha, *beta);
	*delta_z = tj->_aero_model->GetBalanceDeltaZ(t, mach, *alpha, *beta);

	//fprintf(tj->f_zhixin, "%15.10f %15.10f %15.10f\n", t, y[three_degree_trajectory::Y_Index], y[three_degree_trajectory::Z_Index]);
};



//����ˮƽ���ڵ�dq/dt, ��׼��Ϊ ox�ᣨ��������ϵ��
double Proportional_Navigation_ControlModel::calc_dq_h_dt(double dx, double dz, double sigma, double sigma_t, double V, double V_t, double& q_h, double& eta_h, double& r_h) {

	//double q_h;
	if (dz < 0) { //dz = Zt-Zm ��˵����ʱ�����Ѿ�������Ŀ�����ں�������
		if (abs(dx) < 0.00001)
			q_h = 90 / RAD;
		else if (dx > 0)//��һ����
			q_h = atan( fabs(dz / dx) );
		else//�ڶ�����
			q_h = 180/RAD - atan(fabs(dz / dx)) ;
	}
	else { //dz >= 0 
		if (abs(dx) < 0.00001) //=0
			q_h = -90 / RAD;
		else if (dx > 0)//��������
			q_h = -atan(fabs(dz / dx));
		else //dx < 0 , ��������
			q_h = - ( 180/RAD - atan( fabs(dz / dx) ) ) ;
	}

	eta_h = q_h - sigma;
	double eta_t = q_h - sigma_t;
	r_h = sqrt(dx*dx + dz*dz);
	//return (V*sin(eta) - v_t_x*sin(eta_t)) / r;
	double delta_M = V * sin(eta_h); //�����ٶȷ����ڻ�׼�߷����ϵ�λ��
	if (fabs(delta_M) < 0.000001)
		delta_M = 0;//ż����С����ɵ�ƫ��(��Ҫ����q_h -sigma����)�����������
	double delta_T = V_t * sin(eta_t); //Ŀ���ٶȷ����ڻ�׼�߷����ϵ�λ��
	//return (V*sin(eta_h) - V_t*sin(eta_t)) / r_h;
	return (delta_M - delta_T) / r_h;

}


double Proportional_Navigation_ControlModel::calc_dq_v_dt(double dx, double dy, double sigma, double sigma_t, double V, double V_t, double& q) {
	if (dy < 0) { //������һ����� dy = Yt - Ym < 0
		if (abs(dx) < 0.00001)
			q = -90 / RAD;
		else if (dx > 0) //��������
			q = -atan(fabs(dy / dx));
		else //dx<0, ��������
			q = -(180/RAD - atan( fabs(dy / dx) ) );
	}
	else{ //dy >= 0 , �ؿյ���һ����� dy = Yt - Ym
		if (abs(dx) < 0.00001)
			q = 90 / RAD;
		else if (dx > 0) //��һ����
			q = atan( fabs(dy / dx ) );
		else //dx < 0 �ڶ�����
			q = 180/RAD - atan( fabs(dy / dx) ) ;
	}
	
	double eta = q - sigma;
	double eta_t = q - sigma_t;
	double r = sqrt(dx*dx + dy*dy);
	//return (V*sin(eta) - v_t_x*sin(eta_t)) / r;
	double delta_M = V*sin(eta);
	double delta_T = V_t*sin(eta_t);
	//return (V*sin(eta) - V_t*sin(eta_t)) / r;
	return (delta_M - delta_T) / r;
}

void Proportional_Navigation_ControlModel::control_equations(double t, double dt, double y[], double * delta_y, double * delta_z, double * alpha, double * beta, double * gamma_v, three_degree_trajectory * tj)
{
	if (calc_r(t, y) < distance_of_stop_nav) {
		//������������� ʹ��֮ǰ�Ĺ��أ����ǺͲ໬�����ã�
		*gamma_v = 0;
		*alpha = tj->_alpha;
		*beta = tj->_beta;
		*delta_y = tj->_delta_y; //���˴���ƫ���Ƿ�Ӧ�����¼��㣿
		*delta_z = tj->_delta_z; //���˴���ƫ���Ƿ�Ӧ�����¼��㣿
	}
	else {
		double const& V = y[three_degree_trajectory::V_Index];
		double const& theta = y[three_degree_trajectory::Theta_Index];
		double const& psi_v = y[three_degree_trajectory::Psi_V_Index];
		double const& m = y[three_degree_trajectory::Mass_Index];
		double const& y_ = y[three_degree_trajectory::Y_Index];
		double const& x = y[three_degree_trajectory::X_Index];
		double const& z = y[three_degree_trajectory::Z_Index];

		double t_interp = t - t_start_; //��ģ�Ϳ�ʼ�����ʱ��

		//Ŀ���˶����ɣ���������ϵ
		//double x_t = tj->_target_model->X(t); 
		//double y_t = tj->_target_model->Y(t);
		//double z_t = tj->_target_model->Z(t);

		double x_t = target_movement_.X(t);//x_t_0 + v_t_x*t_interp;
		double y_t = target_movement_.Y(t);//y_t_0;
		double z_t = target_movement_.Z(t);//z_t_0 + v_t_z*t_interp;
		double v_t_x = target_movement_.Vx(t);
		double v_t_y = target_movement_.Vy(t);
		double v_t_z = target_movement_.Vz(t);


		//ˮƽ����(horizontal���±��дΪh)���Զ���׼����Լ��ι�ϵ�ͽ��ٶȷ��̣�����ϵΪ������������ϵ��ԭ��ƽ�Ƶ���������x��z��
		//��׼��Ϊox��
		//double q_h = -atan( (z_t - z) / (x_t - x) ); //Ŀ�귽λ��
		double sigma_h = psi_v; //�����ٶ����׼�߼н�
		//double eta_h = q_h - sigma_h;
		double sigma_t_h = 0;//Ŀ���ٶ����׼�߼н�
		if (v_t_x == 0) {
			if (v_t_z > 0)
				sigma_t_h = -90/RAD;
			else sigma_t_h = 90/RAD;
		}
		else if(v_t_x > 0){ //��һ��������
			sigma_t_h = atan(-v_t_z / v_t_x);
		}
		else { //v_t_x < 0, �ڶ���������
			if (v_t_z < 0) //�ڶ�����
				sigma_t_h = 180 / RAD - atan(v_t_z / v_t_x);
			else //��������
				sigma_t_h = -180 / RAD - atan(v_t_z / v_t_x);
		}
		//double eta_t_h = q_h - sigma_t_h;
		//double r_h = sqrt((x_t - x)*(x_t - x) + (z_t - z)*(z_t - z));
		double V_h = V*cos(theta); //�����ٶ���ˮƽ���ͶӰ����������ǳ���-90��ʱ��cos(theta)Ϊ���� ͶӰ���ٶȣ�ʸ������Ϊ���� ��ʵ��ʱ�򵯵�ƫ��Ӧ������ת���ˣ�
		double V_t_h = sqrt(v_t_x*v_t_x + v_t_z*v_t_z); //Ŀ���ٶ���ˮƽ���ͶӰ
		double q_h;//ˮƽ���ڵ�Ŀ�귽λ��
		double eta_h;
		double r_h;
		double dq_h_dt = calc_dq_h_dt(x_t - x, z_t - z, sigma_h, sigma_t_h, V_h, V_t_h, q_h, eta_h, r_h); //ˮƽ����Ŀ���ߵ���ת���ٶ�

		if (K == 1)//�ٶ�׷�ٷ�������������ƹ��ɣ� ��sigma����Ϊָ��Ŀ�귽�� = Ŀ�귽λ��q
		{
			y[three_degree_trajectory::Psi_V_Index] = q_h;
			sigma_h = q_h;
			dq_h_dt = calc_dq_h_dt(x_t - x, z_t - z, sigma_h, sigma_t_h, V_h, V_t_h, q_h, eta_h, r_h); //ˮƽ����Ŀ���ߵ���ת���ٶ�
		}

		//���������ٶ�ʸ��������ƽ���ڣ�vertical,�±��дΪv)�Զ���׼����Լ��ι�ϵ�ͽ��ٶȷ��̣���׼��Ϊˮƽ��
		//����ƽ������������ˮƽ���ڵ���Щ�ǶȺ;���ֵ
		//if (abs(x_t - x) < 0.0000000001)
		//	q_h = 90 / RAD;
		//else
		//	q_h = -atan((z_t - z) / (x_t - x));
		//double eta_h = q_h - sigma_h; //ˮƽ���ڵĵ���ǰ�ý�
		//double r_h = sqrt((x_t - x)*(x_t - x) + (z_t - z)*(z_t - z)); //ˮƽ���ڵĵ�Ŀ����

		double sigma_v = theta;//�����ٶ����׼�߼н�
		double sigma_t_v = 0;//Ŀ���ٶ����׼�߼нǣ���Ϊ���ں�����Ŀ����ˮƽ�����˶�����˼н�Ϊ0
		double V_v = V; //�����ٶ�������ƽ��ͶӰ����Ϊ����ƽ����������ٶ�ʸ����������С����V
		double V_t_v = v_t_x*cos(psi_v) - v_t_z*sin(psi_v); //Ŀ���ٶ�������ƽ���ͶӰ
		//! r_h*cos(eta_h) һֱΪ���� ���ǲ��Ե� 
		//double flag_dx = (x_t - x) / abs(x_t - x);
		//if (flag_dx <= 0) {
		//	int dubug = 10;
		//}
		double dx_v = r_h*cos(eta_h);//������ƽ���ڵ�dx ,  ��������eta_h������ ��Ŀǰ�о�û�д��󣿣� ����û��
		double q_v = 0;//����ƽ���ڵ�Ŀ�귽λ��
		double dq_v_dt = calc_dq_v_dt(dx_v, y_t - y_, sigma_v, sigma_t_v, V_v, V_t_v, q_v); //����ƽ����Ŀ���ߵ���ת���ٶ�

		if (K == 1)//�ٶ�׷�ٷ�������������ƹ��ɣ� ��sigma����Ϊָ��Ŀ�귽�� = Ŀ�귽λ��q
		{
			y[three_degree_trajectory::Theta_Index] = q_v;
			sigma_v = q_v;
			dq_v_dt = calc_dq_v_dt(dx_v, y_t - y_, sigma_v, sigma_t_v, V_v, V_t_v, q_v); //����ƽ����Ŀ���ߵ���ת���ٶ�
		}

		////�Զ���׼������˶�����
		////��׼������Ϊ���������ڵģ�ˮƽ��
		//double d_y = y_t - y_;
		//double d_x = x_t - x;
		//double q_x = atan( d_y/d_x); //����ƽ���Ŀ���߷�λ��
		//double sigma_target = 0;//Ŀ���ٶ������׼�߼н�
		//double eta_target_x = q_x-sigma_target; //Ŀ��ǰ�ý�
		//double sigma_x = theta;//�����ٶ������׼�߼н�
		//double eta_x = q_x - sigma_x; //����ǰ�ý� 
		//double r_x = sqrt(d_y*d_y + d_x*d_x); //������ƽ��ĵ�Ŀ����
		//double dq1_dt = (V*sin(eta_x) - v_t_x*sin(eta_target_x)) /r_x;

		//������������
		double dtheta_dt = K * dq_v_dt; //������Ǳ仯��
		double dpsiv_dt = K * dq_h_dt; //����ƫ�Ǳ仯��


		*gamma_v = 0;

		//��������
		double altitude = y[three_degree_trajectory::Y_Index]; //��ǰ�����߶�
		double sonic = g_atmosphere_model.GetSonic(altitude);
		double rho = g_atmosphere_model.GetRHO(altitude);
		double mach = V / sonic;
		//double S = GetS(tj->aero_data);
		double S = tj->_aero_model->GetS();
		double q = 0.5 * rho * V * V;
		double P = tj->_thrust_model->get_current_thrust(t);

		double previous_beta = tj->_beta;
		double previous_alpha = tj->_alpha;
		*alpha = calc_alphaB_by_dtheta_dt(t, tj->_aero_model, mach, q, S, previous_beta, *gamma_v, P, m, V, dtheta_dt, dpsiv_dt, theta, previous_alpha);//)
		*beta = calc_betaB_by_dpsi_v_dt(t, tj->_aero_model, mach, q, S, *alpha, *gamma_v, P, m, V, dtheta_dt, dpsiv_dt, theta, previous_beta);
		//*alpha = calc_alphaB_by_dtheta_dt(tj->aero_data, mach, q, S, previous_beta, *gamma_v, P, m, V, dtheta_dt, dpsiv_dt, theta, previous_alpha);//)
		//*beta = calc_betaB_by_dpsi_v_dt(tj->aero_data, mach, q, S, *alpha, *gamma_v, P, m, V, dtheta_dt, dpsiv_dt, theta, previous_beta);

		//ƽ���ϵʽ�����ݵ�ǰ�Ĺ��ǺͲ໬�ǣ����Ӧ��ƽ���ƫ��
		*delta_y = tj->_aero_model->GetBalanceDeltaY(t, mach, *alpha, *beta);
		*delta_z = tj->_aero_model->GetBalanceDeltaZ(t, mach, *alpha, *beta);
		//*delta_y = GetBalanceDeltaY_Rad(tj->aero_data, mach, *alpha, *beta);
		//*delta_z = GetBalanceDeltaZ_Rad(tj->aero_data, mach, *alpha, *beta);

		//q_h,ˮƽ���ڵ�Ŀ���߷�λ�ǣ���Ŀ���߽ǣ���q_v������ƽ���ڵ�Ŀ���߷�λ�ǣ���Ŀ���߽ǣ�
		//dq_h_dt , dq_v_dt
		//(q_h-(sigma_h-beta))  , ��q_v-sigma_v-alpha)
		tj->q_h =  q_h;
		tj->dq_h_dt = dq_h_dt;
		tj->q_v = q_v;
		tj->dq_v_dt = dq_v_dt;
		tj->q_m_h = q_h - (sigma_h - *beta);
		tj->q_m_v = q_v  - (sigma_v + *alpha);

	}

}

double Proportional_Navigation_ControlModel::calc_r(double t, double states[]) {
	double& x = states[three_degree_trajectory::X_Index];
	double& y = states[three_degree_trajectory::Y_Index];
	double& z = states[three_degree_trajectory::Z_Index];

	//double dx = x - x_t(t);
	//double dy = y - y_t(t);
	//double dz = z - z_t(t);

	double dx = x - target_movement_.X(t);
	double dy = y - target_movement_.Y(t);
	double dz = z - target_movement_.Z(t);

	return sqrt(dx*dx + dy*dy + dz*dz);

}

bool Proportional_Navigation_ControlModel::is_trajectory_end(double t, double states[]) {
	double r = calc_r(t, states);

	if (r < distance_of_end)
		return true;

	return false;
}

//void ThreePoint_Navigation_ControlModel::control_equations(double t, double dt, double y[], double* delta_y, double* delta_z, double* alpha, double*beta, double* gamma_v, three_degree_trajectory* tj) {
//
//}
//
////�����Ƿ���Խ��������絼���Ѿ�����Ŀ�꣩����������ʱһ�㲻�жϵ����Ƿ����
//bool ThreePoint_Navigation_ControlModel::is_trajectory_end(double t, double y[]) {
//	return false;
//}
//void ThreePoint_Navigation_ControlModel::Load(Name2Value& n2vs) {
//
//}

//���㷨����
double Threepoint_Navigation_ControlModel::calc_r(double t, double states[]) {
	double& x = states[three_degree_trajectory::X_Index];
	double& y = states[three_degree_trajectory::Y_Index];
	double& z = states[three_degree_trajectory::Z_Index];
	double x_t = target_movement_.X(t);//x_t_0 + v_t_x*t_interp;
	double y_t = target_movement_.Y(t);//y_t_0;//���ں���������Ŀ��ֻ��ˮƽ���˶�
	double z_t = target_movement_.Z(t);//z_t_0 + v_t_z*t_interp;

	double dx = x - x_t;
	double dy = y - y_t;
	double dz = z - z_t;

	return sqrt(dx*dx + dy*dy + dz*dz);

}

bool Threepoint_Navigation_ControlModel::is_trajectory_end(double t, double states[]) {
	double r = calc_r(t, states);

	if (r < distance_of_end)
		return true;

	double const& x = states[three_degree_trajectory::X_Index];

	double x_t = target_movement_.X(t);//x_t_0 + v_t_x*t_interp;

	if (x > x_t)
		return true;



	return false;
}

void Threepoint_Navigation_ControlModel::control_equations(double t, double dt, double y[], double * delta_y, double * delta_z, double * alpha, double * beta, double * gamma_v, three_degree_trajectory * tj)
{
	if (calc_r(t, y) < distance_of_end) {
		//�������㷨������ ʹ��֮ǰ�Ĺ��أ����ǺͲ໬�����ã�
		*gamma_v = 0;
		*alpha = tj->_alpha;
		*beta = tj->_beta;
		*delta_y = tj->_delta_y;
		*delta_z = tj->_delta_z;
	}
	else {
		double const& V = y[three_degree_trajectory::V_Index];
		double const& theta = y[three_degree_trajectory::Theta_Index];
		double const& psi_v = y[three_degree_trajectory::Psi_V_Index];
		double const& m = y[three_degree_trajectory::Mass_Index];
		double const& y_ = y[three_degree_trajectory::Y_Index];
		double const& x = y[three_degree_trajectory::X_Index];
		double const& z = y[three_degree_trajectory::Z_Index];


		double t_interp = t - t_start_; //��ģ�Ϳ�ʼ�����ʱ��


		//Ŀ���˶����ɣ���������ϵ,tʱ�� 
		double x_t = target_movement_.X(t);//x_t_0 + v_t_x*t_interp;
		double y_t = target_movement_.Y(t);//y_t_0;//���ں���������Ŀ��ֻ��ˮƽ���˶�
		double z_t = target_movement_.Z(t);//z_t_0 + v_t_z*t_interp;
		double v_t_x = target_movement_.Vx(t);
		double v_t_y = target_movement_.Vy(t);
		double v_t_z = target_movement_.Vz(t);
		//����վ�˶����ɣ�tʱ��
		double x_c = guidance_station_movement_.X(t);//x_c_0 + v_c_x*t_interp;
		double y_c = guidance_station_movement_.Y(t);//y_c_0 + v_c_y*t_interp;
		double z_c = guidance_station_movement_.Z(t);//z_c_0 + v_c_z*t_interp;
		double v_c_x = guidance_station_movement_.Vx(t);
		double v_c_y = guidance_station_movement_.Vy(t);
		double v_c_z = guidance_station_movement_.Vz(t);


		//ͶӰ

		//Ŀ��
		double x_t_v = x_t*cos(psi_v) - z_t*sin(psi_v);
		double y_t_v = y_t;
		double v_t_x_v = v_t_x*cos(psi_v) - v_t_z *sin(psi_v);
		//double v_t_y_v = 0;//Ŀ��y�����ٶ�Ϊ0
		double v_t_y_v = v_t_y;//Ŀ��y�����ٶ�

						   //�Ƶ�վ
		double x_c_v = x_c*cos(psi_v) - z_c*sin(psi_v);
		double y_c_v = y_c;
		double v_c_x_v = v_c_x*cos(psi_v) - v_c_z *sin(psi_v);
		double v_c_y_v = v_c_y;

		//�Ƕ����㣬Ϊ������������λ��
		double epsilon_h = atan2(-(z_t - z_c), (x_t - x_c));//��׼�ߵ���Ŀ����ʱ��Ϊ��
		double epsilon_v = atan2((y_t_v - y_c_v), (x_t_v - x_c_v));


		//����(δ����)
		double x_v = x*cos(psi_v) - z*sin(psi_v);
		double y_v = y_;

		//��������λ��
		double r_missile = sqrt((x - x_c)*(x - x_c) + (y_ - y_c)*(y_ - y_c) + (z - z_c)*(z - z_c));
		y[three_degree_trajectory::X_Index] = r_missile*cos(epsilon_v)*cos(epsilon_h) + x_c;
		y[three_degree_trajectory::Y_Index] = r_missile*sin(epsilon_v) + y_c;
		y[three_degree_trajectory::Z_Index] = -r_missile*cos(epsilon_v)*sin(epsilon_h) + z_c;
		//y[three_degree_trajectory::Y_Index] = sqrt(x_v*x_v + y_v*y_v)*cos(epsilon_v - atan2(y_v - y_c_v, x_v - x_c_v))*sin(epsilon_v) + y_c_v;
		//y[three_degree_trajectory::X_Index] = sqrt(x*x + z*z)*cos(epsilon_h - atan2(-z + z_c, x - x_c))*cos(epsilon_h) + x_c;//tanӦ��Ϊ��ֵ
		//y[three_degree_trajectory::Z_Index] = -sqrt(x*x + z*z)*cos(epsilon_h - atan2(-z + z_c, x - x_c))*sin(epsilon_h) + z_c;//z��ֵ�Ƕ�Ϊ��
		double const& y1_ = y[three_degree_trajectory::Y_Index];
		double const& x1 = y[three_degree_trajectory::X_Index];
		double const& z1 = y[three_degree_trajectory::Z_Index];

		//������������
		x_v = x1*cos(psi_v) - z1*sin(psi_v);
		y_v = y1_;

		//����
		double altitude = y[three_degree_trajectory::Y_Index]; //��ǰ�����߶�
		double sonic = g_atmosphere_model.GetSonic(altitude);
		double rho = g_atmosphere_model.GetRHO(altitude);
		double mach = V / sonic;
		double S = tj->_aero_model->GetS();
		double q = 0.5 * rho * V * V;
		double P = tj->_thrust_model->get_current_thrust(t);

		double previous_beta = tj->_beta;
		double previous_alpha = tj->_alpha;

		//����dpsiv_dt��Ҫ��ֵ
		double V_h = V*cos(theta);
		double v_c_h = sqrt(v_c_x*v_c_x + v_c_z*v_c_z);
		double v_t_h = sqrt(v_t_x*v_t_x + v_t_z*v_t_z);
		double psiv_c = atan2(-v_c_z, v_c_x);//psiv��ʱ��Ϊ��
		double psiv_t = atan2(-v_t_z, v_t_x);//psiv��ʱ��Ϊ��
		double R_h = sqrt((x - x_c)*(x - x_c) + (z - z_c)*(z - z_c));
		double Rt_h = sqrt((x_t - x_c)*(x_t - x_c) + (z_t - z_c)*(z_t - z_c));

		double depsilon_h_dt = (V_h*sin(psi_v - epsilon_h) - v_c_h*sin(psiv_c - epsilon_h)) / R_h;
		double dR_h_dt = V_h*cos(psi_v - epsilon_h) - v_c_h*cos(psiv_c - epsilon_h);
		double dRt_h_dt = v_t_h*cos(psiv_t - epsilon_h) - v_c_h*cos(psiv_c - epsilon_h);

		//double beta_t;
		//for (beta_t = -30; beta_t <= 30; beta_t = beta_t + 0.1)
		//{
		//	double beta_test = beta_t / 57.3;

		//}



		//����dtehta_dt��Ҫ��ֵ
		double R_v = sqrt((x_v - x_c_v)*(x_v - x_c_v) + (y_v - y_c_v)*(y_v - y_c_v));
		double Rt_v = sqrt((x_t_v - x_c_v)*(x_t_v - x_c_v) + (y_t_v - y_c_v)*(y_t_v - y_c_v));
		double v_c_v = sqrt(v_c_x_v*v_c_x_v + v_c_y_v*v_c_y_v);
		double v_t_v = sqrt(v_t_x_v*v_t_x_v + v_t_y_v*v_t_y_v);
		double theta_t_v = atan2(v_t_y_v, v_t_x_v);
		double theta_c_v = atan2(v_c_y_v, v_c_x_v);

		double depsilon_v_dt = (V*sin(theta - epsilon_v) - v_c_v*sin(theta_c_v - epsilon_v)) / R_v;
		double dR_v_dt = V*cos(theta - epsilon_v) - v_c_v*cos(theta_c_v - epsilon_v);
		double dRt_v_dt = v_t_v*cos(theta_t_v - epsilon_v) - v_c_v*cos(theta_c_v - epsilon_v);
		double depsilon_t_v_dt= (v_t_v*sin(theta_t_v - epsilon_v) - v_c_v*sin(theta_c_v - epsilon_v)) / Rt_v;

		//double alpha_t;
		double test = 100000000000000;
		double alpha_now;
		//double test_t;
		double alpha_test1 = -10 / RAD;
		double alpha_test2 = 10 / RAD;
		double test1, test2;
		double X_test, Y_test, Z_test;
		do
		{
			alpha_now = (alpha_test1 + alpha_test2) / 2;

			double delta_z_test = tj->_aero_model->GetBalanceDeltaZ(t, mach, alpha_test1, previous_beta);
			double delta_y_test = tj->_aero_model->GetBalanceDeltaY(t, mach, alpha_test1, previous_beta);
			tj->_aero_model->calc_aerodynamic_force(t, V, sonic, rho, alpha_test1, previous_beta, tj->_delta_x, delta_y_test, delta_z_test, X_test, Y_test, Z_test);
			double dtheta1, dtheta2, dV;
			dtheta1 = (P*sin(alpha_test1) + Y_test - m*G*cos(theta)) / m / V;
			dV = (P*cos(alpha_test1) - X_test - m*G*sin(theta)) / m;
			dtheta2 = (2 - 2 * R_v*dRt_v_dt / Rt_v / dR_v_dt - R_v*dV / dR_v_dt / V  )*depsilon_t_v_dt;
			test1 = dtheta1 - dtheta2;//P*sin(alpha_test1) + Y_test + (P*cos(alpha_test1) - X_test)*tan(theta - epsilon_v) - 2 * m*depsilon_v_dt*(dR_v_dt*Rt_v - R_v*dRt_v_dt) / (Rt_v*cos(theta - epsilon_v)) - m*G*cos(theta) - m*G*sin(theta)*tan(theta - epsilon_v);

			delta_z_test = tj->_aero_model->GetBalanceDeltaZ(t, mach, alpha_now, previous_beta);
			delta_y_test = tj->_aero_model->GetBalanceDeltaY(t, mach, alpha_now, previous_beta);
			tj->_aero_model->calc_aerodynamic_force(t, V, sonic, rho, alpha_now, previous_beta, tj->_delta_x, delta_y_test, delta_z_test, X_test, Y_test, Z_test);			
			dtheta1 = (P*sin(alpha_now) + Y_test - m*G*cos(theta)) / m / V;
			dV = (P*cos(alpha_now) - X_test - m*G*sin(theta)) / m;
			dtheta2 = (2 - 2 * R_v*dRt_v_dt / Rt_v / dR_v_dt - R_v*dV / dR_v_dt / V )*depsilon_t_v_dt;
			test2 = dtheta1 - dtheta2;
			//test2 = P*sin(alpha_now) + Y_test + (P*cos(alpha_now) - X_test)*tan(theta - epsilon_v) - 2 * m*depsilon_v_dt*(dR_v_dt*Rt_v - R_v*dRt_v_dt) / (Rt_v*cos(theta - epsilon_v)) - m*G*cos(theta) - m*G*sin(theta)*tan(theta - epsilon_v);

			if (test1*test2 > 0)
				alpha_test1 = alpha_now;
			else
				alpha_test2 = alpha_now;
		} while (fabs(test2) > 0.0001 && fabs(alpha_test1-alpha_test2) > 0.001/RAD);
		/*
		for (alpha_t = -30; alpha_t <= 30; alpha_t = alpha_t + 0.1)
		{
			double alpha_test = alpha_t / 57.3;
			double delta_z_test = tj->_aero_model->GetBalanceDeltaZ(t, mach, alpha_test, previous_beta);
			double delta_y_test = tj->_aero_model->GetBalanceDeltaY(t, mach, alpha_test, previous_beta);
			double X_test, Y_test, Z_test;
			tj->_aero_model->calc_aerodynamic_force(t, V, sonic, rho, alpha_test, previous_beta, tj->_delta_x, delta_y_test, delta_z_test, X_test, Y_test, Z_test);
			test_t = P*sin(alpha_test) + Y_test + (P*cos(alpha_test) - X_test)*tan(theta - epsilon_v) - 2 * m*depsilon_v_dt*(dR_v_dt*Rt_v - R_v*dRt_v_dt) / (Rt_v*cos(theta - epsilon_v)) - m*G*cos(theta) - m*G*sin(theta)*tan(theta - epsilon_v);
			if (abs(test_t) < test)
			{
				test = abs(test_t);
				alpha_now = alpha_t;
			}
			if (abs(test_t) < 0.1)
			{
				alpha_now = alpha_t;
				break;
			}
		}
		*/
		*alpha = alpha_now;
		*beta = 0;
		*gamma_v = 0;

		//��������

		//*alpha = tj->_aero_model->calc_alphaB_by_dtheta_dt_(t, mach, q, S, previous_beta, *gamma_v, P, m, V, dtheta_dt, dpsiv_dt, theta, previous_alpha);
		//*beta = tj->_aero_model->calc_betaB_by_dpsi_v_dt_(t, mach, q, S, *alpha, *gamma_v, P, m, V, dtheta_dt, dpsiv_dt, theta, previous_beta);
		//ƽ���ϵʽ�����ݵ�ǰ�Ĺ��ǺͲ໬�ǣ����Ӧ��ƽ���ƫ��
		*delta_z = tj->_aero_model->GetBalanceDeltaZ(t, mach, *alpha, *beta);
		*delta_y = tj->_aero_model->GetBalanceDeltaY(t, mach, *alpha, *beta);

	}

}

double Threepoint_Navigation_ControlModel::calc_depsilon_dt(double dx, double dy, double theta_c, double theta_t, double V_c, double V_t) {
	double epsilon;
	if (abs(dx) < 0.0000001)
		epsilon = 90 / RAD;
	else
		epsilon = atan(dy / dx);
	double eta_c = epsilon - theta_c;
	double eta_t = epsilon - theta_t;
	double r = sqrt(dx*dx + dy*dy);
	return (V_c*sin(eta_c) - V_t*sin(eta_t)) / r;
}

//��
double Qianzhi_Navigation_ControlModel::calc_r(double t, double states[]) {
	double& x = states[three_degree_trajectory::X_Index];
	double& y = states[three_degree_trajectory::Y_Index];
	double& z = states[three_degree_trajectory::Z_Index];
	double x_t = target_movement_.X(t);//x_t_0 + v_t_x*t_interp;
	double y_t = target_movement_.Y(t);//y_t_0;//���ں���������Ŀ��ֻ��ˮƽ���˶�
	double z_t = target_movement_.Z(t);//z_t_0 + v_t_z*t_interp;

	double dx = x - x_t;
	double dy = y - y_t;
	double dz = z - z_t;

	return sqrt(dx*dx + dy*dy + dz*dz);

	//double& x = states[three_degree_trajectory::X_Index];
	//double& y = states[three_degree_trajectory::Y_Index];
	//double& z = states[three_degree_trajectory::Z_Index];

	//double dx = x - x_t(t);
	//double dy = y - y_t(t);
	//double dz = z - z_t(t);

	//return sqrt(dx*dx + dy*dy + dz*dz);

}

bool Qianzhi_Navigation_ControlModel::is_trajectory_end(double t, double states[]) {
	double r = calc_r(t, states);

	if (r < distance_of_end)
		return true;


	double const& x = states[three_degree_trajectory::X_Index];

	double x_t = target_movement_.X(t);//x_t_0 + v_t_x*t_interp;

	if (x > x_t)
		return true;

	if (states[three_degree_trajectory::Y_Index] <= 0)
		return true;

	double V = states[three_degree_trajectory::V_Index];
	if ( V <= 20 )
		return true;

	return false;
}

extern double g_qianzhi_alpha_limit;
extern double g_qianzhi_beta_limit;
extern int g_qianzhi_ajust_flag;

void Qianzhi_Navigation_ControlModel::control_equations(double t, double dt, double y[], double * delta_y, double * delta_z, double * alpha, double * beta, double * gamma_v, three_degree_trajectory * tj)
{
	//if (calc_r(t, y) < distance_of_end) {
	//	//������������� ʹ��֮ǰ�Ĺ��أ����ǺͲ໬�����ã�
	//	*gamma_v = 0;
	//	*alpha = tj->_alpha;
	//	*beta = tj->_beta;
	//	*delta_y = tj->_delta_y;
	//	*delta_z = tj->_delta_z;
	//}
	//else 
	{
		double const& V = y[three_degree_trajectory::V_Index];
		double const& theta = y[three_degree_trajectory::Theta_Index];
		double const& psi_v = y[three_degree_trajectory::Psi_V_Index];
		double const& m = y[three_degree_trajectory::Mass_Index];
		double const& y_ = y[three_degree_trajectory::Y_Index];
		double const& x = y[three_degree_trajectory::X_Index];
		double const& z = y[three_degree_trajectory::Z_Index];


		double t_interp = t - t_start_; //��ģ�Ϳ�ʼ�����ʱ��


		//Ŀ���˶����ɣ���������ϵ,tʱ�� 
		double x_t = target_movement_.X(t);//x_t_0 + v_t_x*t_interp;
		double y_t = target_movement_.Y(t);//y_t_0;//���ں���������Ŀ��ֻ��ˮƽ���˶�
		double z_t = target_movement_.Z(t);//z_t_0 + v_t_z*t_interp;
		double v_t_x = target_movement_.Vx(t);
		double v_t_y = target_movement_.Vy(t);
		double v_t_z = target_movement_.Vz(t);
		//����վ�˶����ɣ�tʱ��
		double x_c = guidance_station_movement_.X(t);//x_c_0 + v_c_x*t_interp;
		double y_c = guidance_station_movement_.Y(t);//y_c_0 + v_c_y*t_interp;
		double z_c = guidance_station_movement_.Z(t);//z_c_0 + v_c_z*t_interp;
		double v_c_x = guidance_station_movement_.Vx(t);
		double v_c_y = guidance_station_movement_.Vy(t);
		double v_c_z = guidance_station_movement_.Vz(t);

		//ͶӰ

		//Ŀ��

		double x_t_v = x_t*cos(psi_v) - z_t*sin(psi_v);
		double y_t_v = y_t;
		double v_t_x_v = v_t_x*cos(psi_v) - v_t_z *sin(psi_v);
		//double v_t_y_v = 0;//Ŀ��y�����ٶ�Ϊ0
		double v_t_y_v = v_t_y;//Ŀ��y�����ٶ�

						   //�Ƶ�վ
		double x_c_v = x_c*cos(psi_v) - z_c*sin(psi_v);
		double y_c_v = y_c;
		double v_c_x_v = v_c_x*cos(psi_v) - v_c_z *sin(psi_v);
		double v_c_y_v = v_c_y;

		//����������ǰ��
		double x_v = x*cos(psi_v) - z*sin(psi_v);
		double y_v = y_;

		//�Ƕ����㣬Ϊ������������λ��
		double epsilon_h_t = atan2(-(z_t - z_c), (x_t - x_c));//��׼�ߵ���Ŀ����ʱ��Ϊ��
		double epsilon_v_t = atan2((y_t_v - y_c_v), (x_t_v - x_c_v));

		double r_missile = sqrt((x - x_c)*(x - x_c) + (y_ - y_c)*(y_ - y_c) + (z - z_c)*(z - z_c));

		//�����ڿռ��в��ö���ǰ�ýǣ����Էֳ�����ƽ��
		double r_missile_h = sqrt((x - x_c)*(x - x_c) + (z - z_c)*(z - z_c));
		double r_target_h = sqrt((x_t - x_c)*(x_t - x_c) + (z_t - z_c)*(z_t - z_c));
		double r_missile_v = sqrt((x_v - x_c_v)*(x_v - x_c_v) + (y_v - y_c_v)*(y_v - y_c_v));
		double r_target_v = sqrt((x_t_v - x_c_v)*(x_t_v - x_c_v) + (y_t_v - y_c_v)*(y_t_v - y_c_v));
		double delta_epsilon_h = d_epsilon0_h*exp(r_missile_h / (r_missile_h - r_target_h));
		double delta_epsilon_v = d_epsilon0_v*exp(r_missile_v / (r_missile_v - r_target_v));
		if (r_missile_h >= r_target_h || delta_epsilon_h == 0)//�㷨��bug��ָ�����㾫�ȵ�bug
			delta_epsilon_h = 0;
		if (r_missile_v >= r_target_v || delta_epsilon_v == 0)//�㷨��bug��ָ�����㾫�ȵ�bug
			delta_epsilon_v = 0;
		double epsilon_h = epsilon_h_t + delta_epsilon_h;
		double epsilon_v = epsilon_v_t + delta_epsilon_v;

		//if (t > 51.059)
		//{
		//	double dummy = 0;
		//}
		////��������λ��  
		//double test = r_missile*cos(epsilon_v)*cos(epsilon_h) + x_c;

		if (g_qianzhi_ajust_flag != 0) {
			y[three_degree_trajectory::X_Index] = r_missile*cos(epsilon_v)*cos(epsilon_h) + x_c;
			//�޸�X�����������ʽ����ֹ����������XXXX��ԭ��������û��������r_missile���䣬��˵��������쳣���٣�����ʱ�����ٶ��Ѿ�С��Ŀ������ƶ��ٶȣ�����������������
			//y[three_degree_trajectory::X_Index] = r_missile*cos(epsilon_v) + x_c;
			y[three_degree_trajectory::Y_Index] = r_missile*sin(epsilon_v) + y_c;
			y[three_degree_trajectory::Z_Index] = -r_missile*cos(epsilon_v)*sin(epsilon_h) + z_c;
			//���ǺͲ໬�������ƫ����µ�����Ǻ�ƫ����Ĳ�̫�ԣ�Ҳ��Ҫ������
			//����������˵���������㷨������Ŀ�겻���ƶ��������ٶ�Ӧ�����Ƶ�վ---Ŀ������һ�£����
			//�������ԭ������������Ǻ͵���ƫ�ǣ�����ǰ�õ�����С�߶����㷨�������ټ���ǰ�ýǼ��ɡ�
			y[three_degree_trajectory::Theta_Index] = atan2((y_t - y_c), (x_t - x_c)) + delta_epsilon_v;
			y[three_degree_trajectory::Psi_V_Index] = atan2(-(z_t - z_c), (x_t - x_c)) + delta_epsilon_h;//��ʱ��Ϊ��

		}

		double const& y1_ = y[three_degree_trajectory::Y_Index];
		double const& x1 = y[three_degree_trajectory::X_Index];
		double const& z1 = y[three_degree_trajectory::Z_Index];


		//������������
		x_v = x1*cos(psi_v) - z1*sin(psi_v);
		y_v = y1_;

		//����
		double altitude = y[three_degree_trajectory::Y_Index]; //��ǰ�����߶�
		double sonic = g_atmosphere_model.GetSonic(altitude);
		double rho = g_atmosphere_model.GetRHO(altitude);
		double mach = V / sonic;
		double S = tj->_aero_model->GetS();
		double q = 0.5 * rho * V * V;
		double P = tj->_thrust_model->get_current_thrust(t);

		double previous_beta = tj->_beta;
		double previous_alpha = tj->_alpha;

		//����dpsiv_dt��Ҫ��ֵ
		double V_h = V*cos(theta);
		double v_c_h = sqrt(v_c_x*v_c_x + v_c_z*v_c_z);
		double v_t_h = sqrt(v_t_x*v_t_x + v_t_z*v_t_z);
		double psiv_c = atan2(-v_c_z, v_c_x);//psiv��ʱ��Ϊ��
		double psiv_t = atan2(-v_t_z, v_t_x);//psiv��ʱ��Ϊ��
		double R_h = sqrt((x - x_c)*(x - x_c) + (z - z_c)*(z - z_c));
		double Rt_h = sqrt((x_t - x_c)*(x_t - x_c) + (z_t - z_c)*(z_t - z_c));

		double depsilon_t_h_dt = (v_t_h*sin(psiv_t - epsilon_h) - v_c_h*sin(psiv_c - epsilon_h)) / Rt_h;
		double dR_h_dt = V_h*cos(psi_v - epsilon_h) - v_c_h*cos(psiv_c - epsilon_h);
		double dRt_h_dt = v_t_h*cos(psiv_t - epsilon_h) - v_c_h*cos(psiv_c - epsilon_h);
		double d_delta_epsilon_h_dt = (R_h*dRt_h_dt - dR_h_dt*Rt_h) / ((R_h - Rt_h)*(R_h - Rt_h))*delta_epsilon_h;
		double depsilon_h_dt = depsilon_t_h_dt + d_delta_epsilon_h_dt;//�����ķ�λ��΢������Ҫ��Ŀ��ķ�λ��΢�ּ�ǰ�ý�΢�ֵõ��������0

		double cons;//��С�����������㳣��
		cons = -2 * dR_h_dt*R_h*depsilon_h_dt / (R_h*R_h) + 2 * Rt_h*dRt_h_dt*depsilon_t_h_dt / (Rt_h*Rt_h) + 2 * (R_h*dRt_h_dt - dR_h_dt*Rt_h) / ((R_h - Rt_h)*(R_h - Rt_h)*(R_h - Rt_h))*(dR_h_dt - dRt_h_dt)*delta_epsilon_h - (R_h*dRt_h_dt - dR_h_dt*Rt_h) / ((R_h - Rt_h)*(R_h - Rt_h))*d_delta_epsilon_h_dt;

		//���ַ����
		double beta_test1 = -g_qianzhi_beta_limit / RAD;
		double beta_test2 = g_qianzhi_beta_limit / RAD;
		double beta_now;
		double test1, test2;
		double X_test, Y_test, Z_test;
		do
		{
			beta_now = (beta_test1 + beta_test2) / 2;

			double delta_z_test = tj->_aero_model->GetBalanceDeltaZ(t, mach, previous_alpha, beta_test1);
			double delta_y_test = tj->_aero_model->GetBalanceDeltaY(t, mach, previous_alpha, beta_test1);
			tj->_aero_model->calc_aerodynamic_force(t, V, sonic, rho, previous_alpha, beta_test1, tj->_delta_x,delta_y_test, delta_z_test, X_test, Y_test, Z_test);
			double dv, dpsiv;
			dv = (P*cos(previous_alpha)*cos(beta_test1) - X_test - m*G*sin(theta)) / m*cos(theta);
			dpsiv = (-P*cos(previous_alpha)*sin(beta_test1) + Z_test) / (-m*V);
			double ddR;
			ddR = dv*cos(psi_v - epsilon_h) - V_h*dpsiv*sin(psi_v - epsilon_h) + depsilon_h_dt*depsilon_h_dt*R_h;
			test1 = (dv*sin(psi_v - epsilon_h) + V_h*dpsiv*cos(psi_v - epsilon_h)) / R_h - (R_h*depsilon_t_h_dt*depsilon_t_h_dt - ddR*Rt_h) / ((R_h - Rt_h)*(R_h - Rt_h))*delta_epsilon_h + cons;

			delta_z_test = tj->_aero_model->GetBalanceDeltaZ(t, mach, previous_alpha, beta_now);
			delta_y_test = tj->_aero_model->GetBalanceDeltaY(t, mach, previous_alpha, beta_now);
			tj->_aero_model->calc_aerodynamic_force(t, V, sonic, rho, previous_alpha, beta_now, tj->_delta_x, delta_y_test, delta_z_test, X_test, Y_test, Z_test);
			dv = (P*cos(previous_alpha)*cos(beta_now) - X_test - m*G*sin(theta)) / m;
			dpsiv = (-P*cos(previous_alpha)*sin(beta_now) + Z_test) / (-m*V);
			ddR = dv*cos(psi_v - epsilon_h) - V_h*dpsiv*sin(psi_v - epsilon_h) + depsilon_h_dt*depsilon_h_dt*R_h;
			test2 = (dv*sin(psi_v - epsilon_h) + V_h*dpsiv*cos(psi_v - epsilon_h)) / R_h - (R_h*depsilon_t_h_dt*depsilon_t_h_dt - ddR*Rt_h) / ((R_h - Rt_h)*(R_h - Rt_h))*delta_epsilon_h + cons;

			if (test1*test2 > 0)
				beta_test1 = beta_now;
			else
				beta_test2 = beta_now;
		} while (fabs(test2) > 0.0000001 && fabs(beta_test1-beta_test2) > 0.001/RAD);


		//����dtehta_dt��Ҫ��ֵ
		double R_v = sqrt((x_v - x_c_v)*(x_v - x_c_v) + (y_v - y_c_v)*(y_v - y_c_v));
		double Rt_v = sqrt((x_t_v - x_c_v)*(x_t_v - x_c_v) + (y_t_v - y_c_v)*(y_t_v - y_c_v));
		double v_c_v = sqrt(v_c_x_v*v_c_x_v + v_c_y_v*v_c_y_v);
		double v_t_v = sqrt(v_t_x_v*v_t_x_v + v_t_y_v*v_t_y_v);
		double theta_t_v = atan2(v_t_y_v, v_t_x_v);
		double theta_c_v = atan2(v_c_y_v, v_c_x_v);

		double depsilon_t_v_dt = (v_t_v*sin(theta_t_v - epsilon_v) - v_c_v*sin(theta_c_v - epsilon_v)) / Rt_v;
		double dR_v_dt = V*cos(theta - epsilon_v) - v_c_v*cos(theta_c_v - epsilon_v);
		double dRt_v_dt = v_t_v*cos(theta_t_v - epsilon_v) - v_c_v*cos(theta_c_v - epsilon_v);
		double d_delta_epsilon_v_dt = (R_v*dRt_v_dt - dR_v_dt*Rt_v) / ((R_v - Rt_v)*(R_v - Rt_v))*delta_epsilon_v;
		double depsilon_v_dt = depsilon_t_v_dt + d_delta_epsilon_v_dt;//�����ķ�λ��΢������Ҫ��Ŀ��ķ�λ��΢�ּ�ǰ�ý�΢�ֵõ��������0


		cons = -2 * dR_v_dt*R_v*depsilon_v_dt / (R_v*R_v) + 2 * Rt_v*dRt_v_dt*depsilon_t_v_dt / (Rt_v*Rt_v) + 2 * (R_v*dRt_v_dt - dR_v_dt*Rt_v) / ((R_v - Rt_v)*(R_v - Rt_v)*(R_v - Rt_v))*(dR_v_dt - dRt_v_dt)*delta_epsilon_v - (R_v*dRt_v_dt - dR_v_dt*Rt_v) / ((R_v - Rt_v)*(R_v - Rt_v))*d_delta_epsilon_v_dt;

		//���ַ����
		double alpha_test1 = -g_qianzhi_alpha_limit / RAD;
		double alpha_test2 = g_qianzhi_alpha_limit / RAD;
		double alpha_now;

		do
		{
			alpha_now = (alpha_test1 + alpha_test2) / 2;

			double delta_z_test = tj->_aero_model->GetBalanceDeltaZ(t, mach, alpha_test1, previous_beta);
			double delta_y_test = tj->_aero_model->GetBalanceDeltaY(t, mach, alpha_test1, previous_beta);
			tj->_aero_model->calc_aerodynamic_force(t, V, sonic, rho, alpha_test1, previous_beta, tj->_delta_x, delta_y_test, delta_z_test, X_test, Y_test, Z_test);
			double dv, dtheta;
			dv = (P*cos(alpha_test1)*cos(previous_beta) - X_test - m*G*sin(theta)) / m;
			dtheta = (P*sin(alpha_test1) + Y_test - m*G*cos(theta)) / (m*V);
			double ddR;
			ddR = dv*cos(theta - epsilon_v) - V*dtheta*sin(theta - epsilon_v) + depsilon_v_dt*depsilon_v_dt*R_v;
			test1 = (dv*sin(theta - epsilon_v) + V*dtheta*cos(theta - epsilon_v)) / R_v - (R_v*depsilon_t_v_dt*depsilon_t_v_dt - ddR*Rt_v) / ((R_v - Rt_v)*(R_v - Rt_v))*delta_epsilon_v + cons;

			delta_z_test = tj->_aero_model->GetBalanceDeltaZ(t, mach, alpha_now, previous_beta);
			delta_y_test = tj->_aero_model->GetBalanceDeltaY(t, mach, alpha_now, previous_beta);
			tj->_aero_model->calc_aerodynamic_force(t, V, sonic, rho, alpha_now, previous_beta, tj->_delta_x, delta_y_test, delta_z_test, X_test, Y_test, Z_test);
			dv = (P*cos(alpha_now)*cos(previous_beta) - X_test - m*G*sin(theta)) / m;
			dtheta = (P*sin(alpha_now) + Y_test - m*G*cos(theta)) / (m*V);
			ddR = dv*cos(theta - epsilon_v) - V*dtheta*sin(theta - epsilon_v) + depsilon_v_dt*depsilon_v_dt*R_v;
			test2 = (dv*sin(theta - epsilon_v) + V*dtheta*cos(theta - epsilon_v)) / R_v - (R_v*depsilon_t_v_dt*depsilon_t_v_dt - ddR*Rt_v) / ((R_v - Rt_v)*(R_v - Rt_v))*delta_epsilon_v + cons;

			if (test1*test2 > 0)
				alpha_test1 = alpha_now;
			else
				alpha_test2 = alpha_now;
		} while (fabs(test2) > 0.000001 && fabs(alpha_test1 - alpha_test2) > 0.001/RAD);
		//if (t>44.47)
		//{
		//	double aaa = 0;
		//}
		*alpha = alpha_now;
		*beta = beta_now;
		*gamma_v = 0;

		*delta_z = tj->_aero_model->GetBalanceDeltaZ(t, mach, *alpha, *beta);
		*delta_y = tj->_aero_model->GetBalanceDeltaY(t, mach, *alpha, *beta);

		//�ɴ�������Ҳ�����ˣ�ֱ�Ӽ������ù���

		if (g_qianzhi_ajust_flag != 0) {
			//���ؼ��㹫ʽ������������ѧ��p60
			double dtheta_dt = depsilon_v_dt;
			double dpsiv_dt = depsilon_h_dt;
			tj->_ny2 = (V / G)*dtheta_dt + cos(theta);
			tj->_nz2 = -(V / G) * cos(theta) *dpsiv_dt;
		}


	}

}

//�߷����㷨�����°棩
double Gaofei_Navigation_ControlModel::calc_r(double t, double states[]) {
	double& x = states[three_degree_trajectory::X_Index];
	double& y = states[three_degree_trajectory::Y_Index];
	double& z = states[three_degree_trajectory::Z_Index];

	double dx = x - target_movement_.X(t);
	double dy = y - target_movement_.Y(t);
	double dz = z - target_movement_.Z(t);

	return sqrt(dx*dx + dy*dy + dz*dz);

}

bool Gaofei_Navigation_ControlModel::is_trajectory_end(double t, double states[]) {
	double r = calc_r(t, states);

	if (r < distance_of_end)
		return true;

	double const& x = states[three_degree_trajectory::X_Index];
	double x_t =  target_movement_.X(t);//x_t_0 + v_t_x*t_interp;

	if (x > x_t)
		return true;


	return false;
}
void Gaofei_Navigation_ControlModel::control_equations(double t, double dt, double y[], double * delta_y, double * delta_z, double * alpha, double * beta, double * gamma_v, three_degree_trajectory * tj)
{
	if (calc_r(t, y) < distance_of_stop_nav) {
		//������������� ʹ��֮ǰ�Ĺ��أ����ǺͲ໬�����ã�
		*gamma_v = 0;
		*alpha = tj->_alpha;
		*beta = tj->_beta;
		*delta_y = tj->_delta_y;
		*delta_z = tj->_delta_z;
	}
	else {
		double const& V = y[three_degree_trajectory::V_Index];
		double const& theta = y[three_degree_trajectory::Theta_Index];
		double const& psi_v = y[three_degree_trajectory::Psi_V_Index];
		double const& m = y[three_degree_trajectory::Mass_Index];
		double const& y_ = y[three_degree_trajectory::Y_Index];
		double const& x = y[three_degree_trajectory::X_Index];
		double const& z = y[three_degree_trajectory::Z_Index];


		double t_interp = t - t_start_; //��ģ�Ϳ�ʼ�����ʱ��
										//����
		double altitude = y[three_degree_trajectory::Y_Index]; //��ǰ�����߶�
		double sonic = g_atmosphere_model.GetSonic(altitude);
		double rho = g_atmosphere_model.GetRHO(altitude);
		double mach = V / sonic;
		double S = tj->_aero_model->GetS();
		double q = 0.5 * rho * V * V;
		double P = tj->_thrust_model->get_current_thrust(t);

		double X_previous, Y_previous, Z_previous;
		double previous_beta = tj->_beta;
		double previous_alpha = tj->_alpha;
		double delta_z_previous = tj->_aero_model->GetBalanceDeltaZ(t, mach, previous_alpha, previous_beta);
		double delta_y_previous = tj->_aero_model->GetBalanceDeltaY(t, mach, previous_alpha, previous_beta);
		tj->_aero_model->calc_aerodynamic_force(t, V, sonic, rho, previous_alpha, previous_beta, tj->_delta_x, delta_y_previous, delta_z_previous, X_previous, Y_previous, Z_previous);
		double dV_= (P*cos(previous_alpha) - X_previous - m*G*sin(theta)) / m;

		//Ŀ���˶����ɣ���������ϵ,tʱ�� 
		double x_t = target_movement_.X(t);
		double y_t = target_movement_.Y(t);
		double z_t = target_movement_.Z(t);
		double v_t_x = target_movement_.Vx(t);
		double v_t_y = target_movement_.Vy(t);
		double v_t_z = target_movement_.Vz(t);

		double x_c = guidance_station_movement_.X(t);
		double y_c = guidance_station_movement_.Y(t);
		double z_c = guidance_station_movement_.Z(t);
		double v_c_x = guidance_station_movement_.Vx(t);
		double v_c_y = guidance_station_movement_.Vy(t);
		double v_c_z = guidance_station_movement_.Vz(t);

		//ͶӰ

		//Ŀ��
		double x_t_v = x_t*cos(psi_v) - z_t*sin(psi_v);
		double y_t_v = y_t;
		double v_t_x_v = v_t_x*cos(psi_v) - v_t_z *sin(psi_v);
		double v_t_y_v = 0;//Ŀ��y�����ٶ�Ϊ0

						   //�Ƶ�վ
		double x_c_v = x_c*cos(psi_v) - z_c*sin(psi_v);
		double y_c_v = y_c;
		double v_c_x_v = v_c_x*cos(psi_v) - v_c_z *sin(psi_v);
		double v_c_y_v = v_c_y;

		//����ǰ�ý��ٶȺ�ǰ�ýǼ��ٶ�
		double PI = 3.1415926;

		double x_t_0 = target_movement_.X(0);
		double x_c_0 = guidance_station_movement_.X(0);
		double R_hf = (x_t_0 - x_c_0)*0.6;
		//double t_hf;// = interp11(arry_T, arry_shecheng, x_t_0*0.6, false);
		double delta_epxilong, ddelta_epxilong, dddelta_epxilong, Rf, dRf, ddRf, h_hf, dh_hf, ddh_hf;
		double H0 = Hd / 1.155556;
		double H_hf = Hd;
		double R1 = R0*1.6;
		double R2 = R0*2.4;
		double R_descend = 400;//�½��η��о���
		double a_h = H0 / R0 / 3 / (R0*R0 - R0*(R1 + R2) + R1*R2);
		double b_h = -1.5*a_h*(R1 + R2);
		double c_h = 3 * a_h*R1*R2;
		double con = H0 - a_h*R0*R0*R0 - b_h*R0*R0 - c_h*R0;
		//double t_R0 = interp11(arry_T, arry_shecheng, R0, false);
		//double t_R2 = interp11(arry_T, arry_shecheng, R2, false);
		double x_m = x - x_c;
		if (x_m <= R0)
		{
			delta_epxilong = atan(H0 / R0);
			Rf = x_m;//interp11(arry_shecheng, arry_T, t_interp, false);
			h_hf = Rf*tan(delta_epxilong);
			ddelta_epxilong = 0;
			dddelta_epxilong = 0;
		}
		else if (x_m <= R2)
		{
			Rf = x_m;// interp11(arry_shecheng, arry_T, t_interp, false);
			dRf = V*cos(theta)-v_c_x;//(interp11(arry_shecheng, arry_T, t_interp + 0.1, false) - Rf) / 0.1;
			ddRf = dV_;// ((interp11(arry_shecheng, arry_T, t_interp + 0.1, false) - Rf) / 0.1 - (-interp11(arry_shecheng, arry_T, t_interp - 0.1, false) + Rf) / 0.1) / 0.1;
			h_hf = a_h*Rf*Rf*Rf + b_h*Rf*Rf + c_h*Rf + con;
			dh_hf = (3 * a_h*Rf*Rf + 2 * b_h*Rf + c_h)*dRf;
			ddh_hf = (6 * a_h*Rf + 2 * b_h)*dRf + (3 * a_h*Rf*Rf + 2 * b_h*Rf + c_h)*ddRf;
			delta_epxilong = atan(h_hf / Rf);
			ddelta_epxilong = (dh_hf*Rf - dRf*h_hf) / Rf / Rf;
			dddelta_epxilong = ((ddh_hf*Rf - ddRf*h_hf)*Rf*Rf - 2 * (dh_hf*Rf - dRf*h_hf)*Rf*dRf) / Rf / Rf / Rf / Rf;
		}
		else if (x_m <= R_hf)
		{
			Rf = x_m;// interp11(arry_shecheng, arry_T, t_interp, false);
			dRf = V*cos(theta)-v_c_x;//(interp11(arry_shecheng, arry_T, t_interp + 0.1, false) - Rf) / 0.1;
			ddRf = dV_;// ((interp11(arry_shecheng, arry_T, t_interp + 0.1, false) - Rf) / 0.1 - (-interp11(arry_shecheng, arry_T, t_interp - 0.1, false) + Rf) / 0.1) / 0.1;
			h_hf = H_hf;
			dh_hf = 0;
			ddh_hf = 0;
			delta_epxilong = atan(h_hf / Rf);
			ddelta_epxilong = (dh_hf*Rf - dRf*h_hf) / Rf / Rf;
			dddelta_epxilong = ((ddh_hf*Rf - ddRf*h_hf)*Rf*Rf - 2 * (dh_hf*Rf - dRf*h_hf)*Rf*dRf) / Rf / Rf / Rf / Rf;
		}
		else if (x_m <= R_hf +R_descend)
		{
			Rf = x_m;// interp11(arry_shecheng, arry_T, t_interp, false);
			dRf = V*cos(theta)-v_c_x;//(interp11(arry_shecheng, arry_T, t_interp + 0.1, false) - Rf) / 0.1;
			ddRf = dV_;// ((interp11(arry_shecheng, arry_T, t_interp + 0.1, false) - Rf) / 0.1 - (-interp11(arry_shecheng, arry_T, t_interp - 0.1, false) + Rf) / 0.1) / 0.1;
			h_hf = H_hf*(0.5 + 0.5*sin(PI / 2. + (x_m - R_hf)*PI / R_descend));
			dh_hf = H_hf*0.5*cos(PI / 2. + (x_m - R_hf)*PI / R_descend)*PI / R_descend;
			ddh_hf = -H_hf*0.5*sin(PI / 2. + (x_m - R_hf)*PI / R_descend)*PI*PI / R_descend / R_descend;
			delta_epxilong = atan(h_hf / Rf);
			ddelta_epxilong = (dh_hf*Rf*dRf - dRf*h_hf) / Rf / Rf;
			dddelta_epxilong = (((ddh_hf*dRf*dRf+dh_hf*ddRf)*Rf - ddRf*h_hf)*Rf*Rf - 2 * (dh_hf*Rf*dRf - dRf*h_hf)*Rf*dRf) / Rf / Rf / Rf / Rf;
		}
		else
		{
			delta_epxilong = 0;
			ddelta_epxilong = 0;
			dddelta_epxilong = 0;
		}
		

		//�Ƕ����㣬Ϊ������������λ��
		double epsilon_h = atan2(-(z_t - z_c), (x_t - x_c));//��׼�ߵ���Ŀ����ʱ��Ϊ��
		double epsilon_v = atan2((y_t_v - y_c_v), (x_t_v - x_c_v)) + delta_epxilong;

		double r_missile = sqrt((x - x_c)*(x - x_c) + (y_ - y_c)*(y_ - y_c) + (z - z_c)*(z - z_c));

		//��������λ��
		y[three_degree_trajectory::X_Index] = r_missile*cos(epsilon_v)*cos(epsilon_h) + x_c;
		y[three_degree_trajectory::Y_Index] = r_missile*sin(epsilon_v) + y_c;
		y[three_degree_trajectory::Z_Index] = -r_missile*cos(epsilon_v)*sin(epsilon_h) + z_c;
		double const& y1_ = y[three_degree_trajectory::Y_Index];
		double const& x1 = y[three_degree_trajectory::X_Index];
		double const& z1 = y[three_degree_trajectory::Z_Index];

		//������������
		double x_v = x1*cos(psi_v) - z1*sin(psi_v);
		double y_v = y1_;

		

		//����dpsiv_dt��Ҫ��ֵ
		double V_h = V*cos(theta);
		double v_c_h = sqrt(v_c_x*v_c_x + v_c_z*v_c_z);
		double v_t_h = sqrt(v_t_x*v_t_x + v_t_z*v_t_z);
		double psiv_c = atan2(-v_c_z, v_c_x);//psiv��ʱ��Ϊ��
		double psiv_t = atan2(-v_t_z, v_t_x);//psiv��ʱ��Ϊ��
		double R_h = sqrt((x - x_c)*(x - x_c) + (z - z_c)*(z - z_c));
		double Rt_h = sqrt((x_t - x_c)*(x_t - x_c) + (z_t - z_c)*(z_t - z_c));

		double depsilon_h_dt = (v_t_h*sin(psiv_t - epsilon_h) - v_c_h*sin(psiv_c - epsilon_h)) / Rt_h;//Ӧ�ô�Ŀ��õ�
		double dR_h_dt = V_h*cos(psi_v - epsilon_h) - v_c_h*cos(psiv_c - epsilon_h);
		double dRt_h_dt = v_t_h*cos(psiv_t - epsilon_h) - v_c_h*cos(psiv_c - epsilon_h);

		//���ַ����
		double beta_test1 = -10 / RAD;
		double beta_test2 = 10 / RAD;
		double beta_now;
		double test1, test2;
		double X_test, Y_test, Z_test;
		do
		{
			beta_now = (beta_test1 + beta_test2) / 2;

			double delta_z_test = tj->_aero_model->GetBalanceDeltaZ(t, mach, previous_alpha, beta_test1);
			double delta_y_test = tj->_aero_model->GetBalanceDeltaY(t, mach, previous_alpha, beta_test1);
			tj->_aero_model->calc_aerodynamic_force(t, V, sonic, rho, previous_alpha, beta_test1, tj->_delta_x, delta_y_test, delta_z_test, X_test, Y_test, Z_test);
			test1 = (P*cos(previous_alpha)*sin(beta_test1) - Z_test) / cos(theta) + (P*cos(previous_alpha)*cos(beta_test1) - X_test)*cos(theta)*tan(psi_v - epsilon_h) - 2 * m*depsilon_h_dt*(dR_h_dt*Rt_h - R_h*dRt_h_dt) / (Rt_h*cos(psi_v - epsilon_h)) - m*G*sin(theta)*cos(theta)*tan(psi_v - epsilon_h);

			delta_z_test = tj->_aero_model->GetBalanceDeltaZ(t, mach, previous_alpha, beta_now);
			delta_y_test = tj->_aero_model->GetBalanceDeltaY(t, mach, previous_alpha, beta_now);
			tj->_aero_model->calc_aerodynamic_force(t, V, sonic, rho, previous_alpha, beta_now, tj->_delta_x, delta_y_test, delta_z_test, X_test, Y_test, Z_test);
			test2 = (P*cos(previous_alpha)*sin(beta_now) - Z_test) / cos(theta) + (P*cos(previous_alpha)*cos(beta_now) - X_test)*cos(theta)*tan(psi_v - epsilon_h) - 2 * m*depsilon_h_dt*(dR_h_dt*Rt_h - R_h*dRt_h_dt) / (Rt_h*cos(psi_v - epsilon_h)) - m*G*sin(theta)*cos(theta)*tan(psi_v - epsilon_h);

			if (test1*test2 > 0)
				beta_test1 = beta_now;
			else
				beta_test2 = beta_now;
		} while (fabs(test2) > 0.0001);


		//����dtehta_dt��Ҫ��ֵ
		double R_v = sqrt((x_v - x_c_v)*(x_v - x_c_v) + (y_v - y_c_v)*(y_v - y_c_v));
		double Rt_v = sqrt((x_t_v - x_c_v)*(x_t_v - x_c_v) + (y_t_v - y_c_v)*(y_t_v - y_c_v));
		double v_c_v = sqrt(v_c_x_v*v_c_x_v + v_c_y_v*v_c_y_v);
		double v_t_v = sqrt(v_t_x_v*v_t_x_v + v_t_y_v*v_t_y_v);
		double theta_t_v = atan2(v_t_y_v, v_t_x_v);
		double theta_c_v = atan2(v_c_y_v, v_c_x_v);

		double depsilon_v_dt = (v_t_v*sin(theta_t_v - epsilon_v) - v_c_v*sin(theta_c_v - epsilon_v)) / Rt_v;//Ӧ�ô�Ŀ��õ�
		double dR_v_dt = V*cos(theta - epsilon_v) - v_c_v*cos(theta_c_v - epsilon_v);
		double dRt_v_dt = v_t_v*cos(theta_t_v - epsilon_v) - v_c_v*cos(theta_c_v - epsilon_v);



		//���ַ����
		double alpha_test1 = -20 / RAD;
		double alpha_test2 = 20 / RAD;
		double alpha_now;

		do
		{
			alpha_now = (alpha_test1 + alpha_test2) / 2;

			double delta_z_test = tj->_aero_model->GetBalanceDeltaZ(t, mach, alpha_test1, previous_beta);
			double delta_y_test = tj->_aero_model->GetBalanceDeltaY(t, mach, alpha_test1, previous_beta);
			tj->_aero_model->calc_aerodynamic_force(t, V, sonic, rho, alpha_test1, previous_beta, tj->_delta_x, delta_y_test, delta_z_test, X_test, Y_test, Z_test);

			double dV1 = (P*cos(alpha_test1) - X_test - m*G*sin(theta)) / m;
			double dtheta1 = (2 - 2 * dRt_v_dt*R_v / (dR_v_dt*Rt_v) - R_v*dV1 / (V*dR_v_dt))*depsilon_v_dt + (2 - R_v*dV1 / (V*dR_v_dt))*ddelta_epxilong + R_v / dR_v_dt*dddelta_epxilong;
			test1 = P*sin(alpha_test1) + Y_test - m*G*cos(theta) - m*V*dtheta1;

			delta_z_test = tj->_aero_model->GetBalanceDeltaZ(t, mach, alpha_now, previous_beta);
			delta_y_test = tj->_aero_model->GetBalanceDeltaY(t, mach, alpha_now, previous_beta);
			tj->_aero_model->calc_aerodynamic_force(t, V, sonic, rho, alpha_now, previous_beta, tj->_delta_x, delta_y_test, delta_z_test, X_test, Y_test, Z_test);
			double dV2 = (P*cos(alpha_now) - X_test - m*G*sin(theta)) / m;
			double dtheta2 = (2 - 2 * dRt_v_dt*R_v / (dR_v_dt*Rt_v) - R_v*dV2 / (V*dR_v_dt))*depsilon_v_dt + (2 - R_v*dV2 / (V*dR_v_dt))*ddelta_epxilong + R_v / dR_v_dt*dddelta_epxilong;
			test2 = P*sin(alpha_now) + Y_test - m*G*cos(theta) - m*V*dtheta2;

			if (test1*test2 > 0)
				alpha_test1 = alpha_now;
			else
				alpha_test2 = alpha_now;
		} while (fabs(test2) > 0.0001);

		*alpha = alpha_now;
		*beta = beta_now;
		*gamma_v = 0;

		*delta_z = tj->_aero_model->GetBalanceDeltaZ(t, mach, *alpha, *beta);
		*delta_y = tj->_aero_model->GetBalanceDeltaY(t, mach, *alpha, *beta);

	}

}