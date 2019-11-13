#include "stdafx.h"
#include "aerodynamic_model.h"
#include "const.h"
#include "utils.h"
#include "../utility.h"
//#include "my_aerodynamic_interp.h"

void LoadThrustAerodynamicModel(ThrustAerodynamicModel* model, std::map<std::string, std::string> n2vs) {

	std::for_each(n2vs.begin(), n2vs.end(), [model](std::pair<std::string, std::string> it) {
		if (it.first == "aero_data_before") {
			model->aero_data_before = aerodynamic_open2(it.second.c_str());
		}
		else if (it.first == "aero_data_after") {
			model->aero_data_after = aerodynamic_open2(it.second.c_str());
		}
		else if (it.first == "t_array") {
			model->t_array= convert2doubles(it.second.c_str()); 
		}
		else if (it.first == "xc_array") {
			model->xc_array = convert2doubles(it.second.c_str());
			model->xc_before_ = model->xc_array.front();
			model->xc_after_ = model->xc_array.back();
		}
		else {
			printf("ThrustAerodynamicModel, δ֪������:%s\n", it.first.c_str());
			prompt_exit(1);
		}
	});

}

AerodynamicModel* AerodynamicModel::ConstructFromFile(double t_start, const char* filename) {
	std::map<std::string, std::string> name_values;
	split_file(filename, '=', name_values);

	if (name_values["type"] == "ThrustAerodynamicModel") {
		ThrustAerodynamicModel* model = new ThrustAerodynamicModel(t_start);
		name_values.erase("type");
		LoadThrustAerodynamicModel(model, name_values);
		return model;
	}

	return NULL;
}

NormalAerodynamicModel::NormalAerodynamicModel(const char* aero_data_dir) {
	_aero_data_path = aero_data_dir;
	aero_data = aerodynamic_open2(aero_data_dir);
}
NormalAerodynamicModel::~NormalAerodynamicModel() {
	if (aero_data)
		aerodynamic_close2(aero_data);
}
void NormalAerodynamicModel::calc_aerodynamic_force(double t, double V, double sonic, double rho,
	double alpha, double beta,
	double delta_x, double delta_y, double delta_z,
	double& X, double& Y, double& Z	)
{
	double S = GetS();
	double mach = V / sonic;
	double q = 0.5 * rho * V * V;

	double CX = GetCX_Rad(aero_data, mach, alpha, beta, delta_x, delta_y, delta_z); //����ϵ��
	double CY = GetCY_Rad(aero_data, mach, alpha, beta, delta_x, delta_y, delta_z);
	double CZ = GetCZ_Rad(aero_data, mach, alpha, beta, delta_x, delta_y, delta_z);
	X = CX*q*S; //����
	Y = CY*q*S; //����
	Z = CZ*q*S;//������

}
	//�������ؼ���
void NormalAerodynamicModel::calc_aerodynamic_torque(double t, double V, double sonic, double rho,
		double alpha, double beta,
		double delta_x, double delta_y, double delta_z,
		double wx, double wy, double wz,
		double& Mx, double& My, double& Mz)
{
	double S = GetS();
	double L = GetL();
	double L_MX = GetL_MX();
	double mach = V / sonic;
	double q = 0.5 * rho * V * V;

	double mx = GetMX_Rad(aero_data, mach, alpha, beta, delta_x);
	double mxwx = GetMXWX0_Rad(aero_data, mach, alpha, beta);
	double my = GetMY_Rad(aero_data, mach, alpha, beta, delta_y);
	double mywy = GetMYWY0_Rad(aero_data, mach, alpha, beta);
	double mz = GetMZ_Rad(aero_data, mach, alpha, beta, delta_z);
	double mzwz = GetMZWZ0_Rad(aero_data, mach, alpha, beta);

	Mx = (mx + mxwx * wx*RAD*L_MX / (2 * V))*q*S*L_MX;
	My = (my + mywy * wy*RAD*L / (2 * V))*q*S*L;
	Mz = (mz + mzwz * wz*RAD*L / (2 * V))*q*S*L;


};

double NormalAerodynamicModel::GetBalanceAlpha(double t, double mach, double beta, double delta_z) {
	return ::GetBalanceAlpha_Rad(this->aero_data, mach, beta, delta_z);
}
double NormalAerodynamicModel::GetBalanceBeta(double t, double mach, double alpha, double delta_z) {
	return ::GetBalanceBeta_Rad(this->aero_data, mach, alpha, delta_z);
}
double NormalAerodynamicModel::GetMaxCLCD_Alpha(double t, double mach, double beta) {
	return ::GetMaxCLCD_Alpha_Rad(this->aero_data, mach, beta);
}
double NormalAerodynamicModel::GetBalanceDeltaZ(double t, double mach, double alpha, double beta) {
	return ::GetBalanceDeltaZ_Rad(this->aero_data, mach, alpha, beta);

}
double NormalAerodynamicModel::GetBalanceDeltaY(double t, double mach, double alpha, double beta) {
	return ::GetBalanceDeltaY_Rad(this->aero_data, mach, alpha, beta);
}
double NormalAerodynamicModel::GetS() {
	return ::GetS(this->aero_data);
}
double NormalAerodynamicModel::GetL() {
	return ::GetL(this->aero_data);
}

double NormalAerodynamicModel::GetL_MX() {
	return ::GetL_MX(this->aero_data);
}


double NormalAerodynamicModel::GetCYBalanceByAlpha(double t, double mach, double alpha, double beta) {
	return ::GetCYBalanceByAlpha_Rad(this->aero_data, mach, alpha, beta);
}
double NormalAerodynamicModel::GetCZBalanceByBeta(double t, double mach, double alpha, double beta) {
	return ::GetCZBalanceByBeta_Rad(this->aero_data, mach, alpha, beta);
}
double NormalAerodynamicModel::GetCX(double t, double mach, double alpha, double beta, double delta_x, double delta_y, double delta_z) {
	return ::GetCX_Rad(this->aero_data, mach, alpha, beta, delta_x, delta_y, delta_z);
}
double NormalAerodynamicModel::GetMX(double t, double mach, double alpha, double beta, double delta_x) {
	return ::GetMX_Rad(this->aero_data, mach, alpha, beta, delta_x);
}
double NormalAerodynamicModel::GetMXWX(double t, double mach, double alpha, double beta, double delta_x) {
	return ::GetMXWX0_Rad(this->aero_data, mach, alpha, beta);
}

double NormalAerodynamicModel::GetCY(double t, double mach, double alpha, double beta, double delta_x, double delta_y, double delta_z) {
	return ::GetCY_Rad(this->aero_data, mach, alpha, beta, delta_x, delta_y, delta_z);
}
double NormalAerodynamicModel::GetMZ(double t, double mach, double alpha, double beta, double delta_z) {
	return ::GetMZ_Rad(this->aero_data, mach, alpha, beta, delta_z);
}
double NormalAerodynamicModel::GetMZWZ(double t, double mach, double alpha, double beta, double delta_z) {
	return ::GetMZWZ0_Rad(this->aero_data, mach, alpha, beta);
}

double NormalAerodynamicModel::GetCZ(double t, double mach, double alpha, double beta, double delta_x, double delta_y, double delta_z) {
	return ::GetCZ_Rad(this->aero_data, mach, alpha, beta, delta_x, delta_y, delta_z);
}
double NormalAerodynamicModel::GetMY(double t, double mach, double alpha, double beta, double delta_y) {
	return ::GetMY_Rad(this->aero_data, mach, alpha, beta, delta_y);
}
double NormalAerodynamicModel::GetMYWY(double t, double mach, double alpha, double beta, double delta_y) {
	return ::GetMYWY0_Rad(this->aero_data, mach, alpha, beta);
}

//������ֵ�����Ϊ��֧�ֶ���ϵ�����㣩
//����ϵ�����������ضԹ��ǡ�������ƫ�ǵĵ���
double NormalAerodynamicModel::GetCY_Alpha(double t, double mach, double alpha, double beta){
	return ::GetCY_per_Alpha_Rad(this->aero_data, mach, alpha, beta);
}
double NormalAerodynamicModel::GetCY_DeltaZ(double t, double mach, double alpha, double beta, double delta_z){
	return ::GetCY_per_DeltaZ_Rad(this->aero_data, mach, alpha, beta, delta_z);
}
double NormalAerodynamicModel::GetMZ_Alpha(double t, double mach, double alpha, double beta){
	return ::GetMZ_per_Alpha_Rad(this->aero_data, mach, alpha, beta);
}
double NormalAerodynamicModel::GetMZ_DeltaZ(double t, double mach, double alpha, double beta, double delta_z){
	return ::GetMZ_per_DeltaZ_Rad(this->aero_data, mach, alpha, beta, delta_z);
}

//������ϵ����ƫ�����ضԲ໬�ǡ�ƫ����ƫ�ǵĵ���
double NormalAerodynamicModel::GetCZ_Beta(double t, double mach, double alpha, double beta){
	return ::GetCZ_per_Beta_Rad(this->aero_data, mach, alpha, beta);
}
double NormalAerodynamicModel::GetCZ_DeltaY(double t, double mach, double alpha, double beta, double delta_y){
	return ::GetCZ_per_DeltaY_Rad(this->aero_data, mach, alpha, beta, delta_y);
}
double NormalAerodynamicModel::GetMY_Beta(double t, double mach, double alpha, double beta){
	return ::GetMY_per_Beta_Rad(this->aero_data, mach, alpha, beta);
}
double NormalAerodynamicModel::GetMY_DeltaY(double t, double mach, double alpha, double beta, double delta_y){
	return ::GetMY_per_DeltaY_Rad(this->aero_data, mach, alpha, beta, delta_y);
}

//��ת���ضԹ�ת��ƫ�ǵĵ���
double NormalAerodynamicModel::GetMX_DeltaX(double t, double mach, double alpha, double beta, double delta_x){
	return ::GetMX_per_DeltaX_Rad(this->aero_data, mach, alpha, beta, delta_x);
}

//���Ŭ˹���ضԲ໬�ǵĵ���
double NormalAerodynamicModel::GetMYWX_Beta_Rad(double t, double mach, double alpha, double beta){
	return::GetMYWX0_per_Beta_Rad(this->aero_data, mach, alpha, beta);
}