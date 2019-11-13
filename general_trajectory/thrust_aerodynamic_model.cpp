#include "stdafx.h"
#include "aerodynamic_model.h"
#include "const.h"

ThrustAerodynamicModel::ThrustAerodynamicModel(double t_start, const char* aero_data_before_dir, const char* aero_data_after_dir){
	this->t_start_ = t_start;
	aero_data_before = aerodynamic_open2(aero_data_before_dir);
	aero_data_after= aerodynamic_open2(aero_data_after_dir);
}
ThrustAerodynamicModel::~ThrustAerodynamicModel() {
	if (aero_data_before)
		aerodynamic_close2(aero_data_before);
	if (aero_data_after)
		aerodynamic_close2(aero_data_after);
}
void ThrustAerodynamicModel::calc_aerodynamic_force(double t, double V, double sonic, double rho,
	double alpha, double beta,
	double delta_x, double delta_y, double delta_z,
	double& X, double& Y, double& Z	)
{
	double S = GetS();
	double mach = V / sonic;
	double q = 0.5 * rho * V * V;

	//因为发动机工作前后气动外形不发生变化，对应力系数是不变的，因此只需要使用aero_data_before计算相应力系数
	//注：阻力系数由于喷口底压影响，在发动机工作期间会变小，此处暂不考虑
	double CX = GetCX_Rad(aero_data_before, mach, alpha, beta, delta_x, delta_y, delta_z); 
	double CY = GetCY_Rad(aero_data_before, mach, alpha, beta, delta_x, delta_y, delta_z);
	double CZ = GetCZ_Rad(aero_data_before, mach, alpha, beta, delta_x, delta_y, delta_z);
	X = CX*q*S; //阻力
	Y = CY*q*S; //升力
	Z = CZ*q*S;//侧向力

}
	//气动力矩计算
void ThrustAerodynamicModel::calc_aerodynamic_torque(double t, double V, double sonic, double rho,
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

	double my_before = GetMY_Rad(aero_data_before, mach, alpha, beta, delta_y);
	double mywy_before = GetMYWY0_Rad(aero_data_before, mach, alpha, beta);
	double mz_before = GetMZ_Rad(aero_data_before, mach, alpha, beta, delta_z);
	double mzwz_before = GetMZWZ0_Rad(aero_data_before, mach, alpha, beta);

	double my_after = GetMY_Rad(aero_data_after, mach, alpha, beta, delta_y);
	double mywy_after = GetMYWY0_Rad(aero_data_after, mach, alpha, beta);
	double mz_after = GetMZ_Rad(aero_data_after, mach, alpha, beta, delta_z);
	double mzwz_after = GetMZWZ0_Rad(aero_data_after, mach, alpha, beta);

	double xc_current = this->calc_x_center(t);
	double my = interp_linear(my_before, my_after, xc_before_, xc_after_, xc_current);
	double mywy = interp_linear(mywy_before, mywy_after, xc_before_, xc_after_, xc_current);
	double mz = interp_linear(mz_before, mz_after, xc_before_, xc_after_, xc_current);
	double mzwz = interp_linear(mzwz_before, mzwz_after, xc_before_, xc_after_, xc_current);

	//mx和mxwx应该跟纵向质心的变化没关系
	double mx= GetMX_Rad(aero_data_after, mach, alpha, beta, delta_x);
	double mxwx= GetMXWX0_Rad(aero_data_after, mach, alpha, beta);

	Mx = (mx + mxwx * wx*RAD*L / (2 * V))*q*S*L_MX;
	My = (my + mywy * wy*RAD*L / (2 * V))*q*S*L;
	Mz = (mz + mzwz * wz*RAD*L / (2 * V))*q*S*L;

};

double ThrustAerodynamicModel::GetBalanceAlpha(double t, double mach, double beta, double delta_z) {
	double alpha_before = ::GetBalanceAlpha_Rad(this->aero_data_before, mach, beta, delta_z);
	double alpha_after= ::GetBalanceAlpha_Rad(this->aero_data_after, mach, beta, delta_z);
	double xc_current = this->calc_x_center(t);
	return interp_linear(alpha_before, alpha_after, xc_before_, xc_after_, xc_current);
}
double ThrustAerodynamicModel::GetBalanceBeta(double t, double mach, double alpha, double delta_y) {
	double before = ::GetBalanceBeta_Rad(this->aero_data_before, mach, alpha, delta_y);
	double after= ::GetBalanceBeta_Rad(this->aero_data_after, mach, alpha, delta_y);
	double xc_current = this->calc_x_center(t);
	return interp_linear(before, after, xc_before_, xc_after_, xc_current);

	//return ::GetBalanceBeta_Rad(this->aero_data, mach, alpha, delta_y);
}
double ThrustAerodynamicModel::GetMaxCLCD_Alpha(double t, double mach, double beta) {
	double before = ::GetMaxCLCD_Alpha_Rad(this->aero_data_before, mach, beta);
	double after= ::GetMaxCLCD_Alpha_Rad(this->aero_data_after, mach, beta);
	double xc_current = this->calc_x_center(t);
	return interp_linear(before, after, xc_before_, xc_after_, xc_current);
	//return ::GetMaxCLCD_Alpha_Rad(this->aero_data, mach, beta);
}
double ThrustAerodynamicModel::GetBalanceDeltaZ(double t, double mach, double alpha, double beta) {
	double before = ::GetBalanceDeltaZ_Rad(this->aero_data_before, mach, alpha, beta);
	double after= ::GetBalanceDeltaZ_Rad(this->aero_data_after, mach, alpha, beta);
	double xc_current = this->calc_x_center(t);
	return interp_linear(before, after, xc_before_, xc_after_, xc_current);
	//return ::GetBalanceDeltaZ_Rad(this->aero_data, mach, alpha, beta);

}
double ThrustAerodynamicModel::GetBalanceDeltaY(double t, double mach, double alpha, double beta) {
	double before = ::GetBalanceDeltaY_Rad(this->aero_data_before, mach, alpha, beta);
	double after= ::GetBalanceDeltaY_Rad(this->aero_data_after, mach, alpha, beta);
	double xc_current = this->calc_x_center(t);
	return interp_linear(before, after, xc_before_, xc_after_, xc_current);
	//return ::GetBalanceDeltaY_Rad(this->aero_data, mach, alpha, beta);
}
double ThrustAerodynamicModel::GetS() {
	return ::GetS(this->aero_data_before);
}
double ThrustAerodynamicModel::GetL() {
	return ::GetL(this->aero_data_before);
}

double ThrustAerodynamicModel::GetL_MX() {
	return ::GetL_MX(this->aero_data_before);
}

double ThrustAerodynamicModel::GetCYBalanceByAlpha(double t, double mach, double alpha, double beta) {
	double before = ::GetCYBalanceByAlpha_Rad(this->aero_data_before, mach, alpha, beta);
	double after= ::GetCYBalanceByAlpha_Rad(this->aero_data_after, mach, alpha, beta);
	double xc_current = this->calc_x_center(t);
	return interp_linear(before, after, xc_before_, xc_after_, xc_current);
}
double ThrustAerodynamicModel::GetCZBalanceByBeta(double t, double mach, double alpha, double beta) {
	double before = ::GetCZBalanceByBeta_Rad(this->aero_data_before, mach, alpha, beta);
	double after  = ::GetCZBalanceByBeta_Rad(this->aero_data_after,  mach, alpha, beta);
	double xc_current = this->calc_x_center(t);
	return interp_linear(before, after, xc_before_, xc_after_, xc_current);
}
double ThrustAerodynamicModel::GetCX(double t, double mach, double alpha, double beta, double delta_x, double delta_y, double delta_z) {
	//阻力系数跟质心变化无关，因此暂时先取工作之前的（实际上因为喷口底压存在，阻力会降低，因此需要特定取值）
	return ::GetCX_Rad(this->aero_data_before, mach, alpha, beta, delta_x, delta_y, delta_z);
}
double ThrustAerodynamicModel::GetMX(double t, double mach, double alpha, double beta, double delta_x) {
	//滚转力矩变化跟纵向质心变化无关，因此取发动机工作之前的（假设侧向质心没有发生变化?）
	return ::GetMX_Rad(this->aero_data_before, mach, alpha, beta, delta_x);
}
double ThrustAerodynamicModel::GetMXWX(double t, double mach, double alpha, double beta, double delta_x) {
	//滚转阻尼力矩变化跟纵向质心变化无关，因此取发动机工作之前的（假设侧向质心没有发生变化?）
	return ::GetMXWX0_Rad(this->aero_data_before, mach, alpha, beta);
}

double ThrustAerodynamicModel::GetCY(double t, double mach, double alpha, double beta, double delta_x, double delta_y, double delta_z) {
	//升力系数跟质心变化无关，因此暂时先取工作之前的
	return ::GetCY_Rad(this->aero_data_before, mach, alpha, beta, delta_x, delta_y, delta_z);
}
double ThrustAerodynamicModel::GetMZ(double t, double mach, double alpha, double beta, double delta_z) {
	double before = ::GetMZ_Rad(this->aero_data_before, mach, alpha, beta, delta_z);
	double after= ::GetMZ_Rad(this->aero_data_after, mach, alpha, beta, delta_z);
	double xc_current = this->calc_x_center(t);
	return interp_linear(before, after, xc_before_, xc_after_, xc_current);
	//return ::GetMZ_Rad(this->aero_data, mach, alpha, beta, delta_z);
}
double ThrustAerodynamicModel::GetMZWZ(double t, double mach, double alpha, double beta, double delta_z) {
	double before = ::GetMZWZ0_Rad(this->aero_data_before, mach, alpha, beta);
	double after= ::GetMZWZ0_Rad(this->aero_data_after, mach, alpha, beta);
	double xc_current = this->calc_x_center(t);
	return interp_linear(before, after, xc_before_, xc_after_, xc_current);
	//return ::GetMZWZ_Rad(this->aero_data, mach, alpha, beta, delta_z);
}

double ThrustAerodynamicModel::GetCZ(double t, double mach, double alpha, double beta, double delta_x, double delta_y, double delta_z) {
	//侧向力系数跟质心变化无关，因此暂时先取工作之前的
	return ::GetCZ_Rad(this->aero_data_before, mach, alpha, beta, delta_x, delta_y, delta_z);
}
double ThrustAerodynamicModel::GetMY(double t, double mach, double alpha, double beta, double delta_y) {
	double before = ::GetMY_Rad(this->aero_data_before, mach, alpha, beta, delta_y);
	double after= ::GetMY_Rad(this->aero_data_after, mach, alpha, beta, delta_y);
	double xc_current = this->calc_x_center(t);
	return interp_linear(before, after, xc_before_, xc_after_, xc_current);
	//return ::GetMY_Rad(this->aero_data, mach, alpha, beta, delta_y);
}
double ThrustAerodynamicModel::GetMYWY(double t, double mach, double alpha, double beta, double delta_y) {
	double before = ::GetMYWY0_Rad(this->aero_data_before, mach, alpha, beta);
	double after= ::GetMYWY0_Rad(this->aero_data_after, mach, alpha, beta);
	double xc_current = this->calc_x_center(t);
	return interp_linear(before, after, xc_before_, xc_after_, xc_current);
	//return ::GetMYWY_Rad(this->aero_data, mach, alpha, beta, delta_y);
}

//计算各种导数（为了支持动力系数计算）
//升力系数、俯仰力矩对攻角、俯仰舵偏角的导数
double ThrustAerodynamicModel::GetCY_Alpha(double t, double mach, double alpha, double beta){
	//升力系数跟质心变化无关，因此暂时先取工作之前的
	return ::GetCY_per_Alpha_Rad(this->aero_data_before, mach, alpha, beta);
}
double ThrustAerodynamicModel::GetCY_DeltaZ(double t, double mach, double alpha, double beta, double delta_z){
	//升力系数跟质心变化无关，因此暂时先取工作之前的
	return ::GetCY_per_DeltaZ_Rad(this->aero_data_before, mach, alpha, beta, delta_z);
}
double ThrustAerodynamicModel::GetMZ_Alpha(double t, double mach, double alpha, double beta){
	double before = ::GetMZ_per_Alpha_Rad(this->aero_data_before, mach, alpha, beta);
	double after= ::GetMZ_per_Alpha_Rad(this->aero_data_after, mach, alpha, beta);
	double xc_current = this->calc_x_center(t);
	return interp_linear(before, after, xc_before_, xc_after_, xc_current);
	//return ::GetMZ_Alpha_Rad(this->aero_data, mach, alpha, beta);
}
double ThrustAerodynamicModel::GetMZ_DeltaZ(double t, double mach, double alpha, double beta, double delta_z){
	double before = ::GetMZ_per_DeltaZ_Rad(this->aero_data_before, mach, alpha, beta, delta_z);
	double after= ::GetMZ_per_DeltaZ_Rad(this->aero_data_after, mach, alpha, beta, delta_z);
	double xc_current = this->calc_x_center(t);
	return interp_linear(before, after, xc_before_, xc_after_, xc_current);
	//return ::GetMZ_DeltaZ_Rad(this->aero_data, mach, alpha, beta, delta_z);
}

//侧向力系数、偏航力矩对侧滑角、偏航舵偏角的导数
double ThrustAerodynamicModel::GetCZ_Beta(double t, double mach, double alpha, double beta){
	return ::GetCZ_per_Beta_Rad(this->aero_data_before, mach, alpha, beta);
}
double ThrustAerodynamicModel::GetCZ_DeltaY(double t, double mach, double alpha, double beta, double delta_y){
	return ::GetCZ_per_DeltaY_Rad(this->aero_data_before, mach, alpha, beta, delta_y);
}
double ThrustAerodynamicModel::GetMY_Beta(double t, double mach, double alpha, double beta){
	double before = ::GetMY_per_Beta_Rad(this->aero_data_before, mach, alpha, beta);
	double after= ::GetMY_per_Beta_Rad(this->aero_data_after, mach, alpha, beta);
	double xc_current = this->calc_x_center(t);
	return interp_linear(before, after, xc_before_, xc_after_, xc_current);
	//return ::GetMY_Beta_Rad(this->aero_data, mach, alpha, beta);
}
double ThrustAerodynamicModel::GetMY_DeltaY(double t, double mach, double alpha, double beta, double delta_y){
	double before = ::GetMY_per_DeltaY_Rad(this->aero_data_before, mach, alpha, beta, delta_y);
	double after= ::GetMY_per_DeltaY_Rad(this->aero_data_after, mach, alpha, beta, delta_y);
	double xc_current = this->calc_x_center(t);
	return interp_linear(before, after, xc_before_, xc_after_, xc_current);
	//return ::GetMY_DeltaY_Rad(this->aero_data, mach, alpha, beta, delta_y);
}

//滚转力矩对滚转舵偏角的导数
double ThrustAerodynamicModel::GetMX_DeltaX(double t, double mach, double alpha, double beta, double delta_x){
	return ::GetMX_per_DeltaX_Rad(this->aero_data_before, mach, alpha, beta, delta_x);
}

//马格努斯力矩对侧滑角的导数
double ThrustAerodynamicModel::GetMYWX_Beta_Rad(double t, double mach, double alpha, double beta){
	double before = ::GetMYWX0_per_Beta_Rad(this->aero_data_before, mach, alpha, beta);
	double after = ::GetMYWX0_per_Beta_Rad(this->aero_data_after, mach, alpha, beta);
	double xc_current = this->calc_x_center(t);
	return interp_linear(before, after, xc_before_, xc_after_, xc_current);
}
