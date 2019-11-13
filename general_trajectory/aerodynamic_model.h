//气动力和力矩计算模型
#pragma once
//#include "..\AeroInterp\aerodynamic_interp\aerodynamic_interp.h"
#include "stdafx.h"
#include "my_aerodynamic_interp.h"
#include "..\interp.h"

class AerodynamicModel {
public:
	static AerodynamicModel* ConstructFromFile(double t_start, const char* filename);

	//气动力计算
	virtual void calc_aerodynamic_force(double t, double V, double sonic, double rho,
		double alpha, double beta,
		double delta_x, double delta_y, double delta_z,
		double& X, double& Y, double& Z
		) = 0;
	//气动力矩计算
	virtual void calc_aerodynamic_torque(double t, double V, double sonic, double rho,
		double alpha, double beta,
		double delta_x, double delta_y, double delta_z,
		double wx, double wy, double wz,
		double& Mx, double& My, double& Mz) = 0;

	virtual double GetS() = 0;
	virtual double GetL() = 0;
	virtual double GetL_MX() = 0;


	virtual double GetMaxAlpha() = 0; //工况中的最大攻角
	virtual double GetMinAlpha() = 0; //工况中的最小攻角

	virtual double GetMaxBeta() = 0; //工况中的最大侧滑角
	virtual double GetMinBeta() = 0; //工况中的最小侧滑角

	virtual double GetBalanceAlpha(double t, double mach, double beta, double delta_z) = 0;
	virtual double GetBalanceBeta(double t, double mach, double beta, double delta_y) = 0;


	virtual double GetMaxCLCD_Alpha(double t, double mach, double beta) = 0;
	virtual double GetBalanceDeltaZ(double t, double mach, double alpha, double beta) = 0;
	virtual double GetBalanceDeltaY(double t, double mach, double alpha, double beta) = 0;


	virtual double GetCYBalanceByAlpha(double t, double mach, double alpha, double beta) = 0;
	virtual double GetCZBalanceByBeta(double t, double mach, double alpha, double beta) = 0;

	virtual double GetCX(double t, double mach, double alpha, double beta, double delta_x, double delta_y, double delta_z) = 0;

	virtual double GetMX(double t, double mach, double alpha, double beta, double delta_x) = 0;
	virtual double GetMXWX(double t, double mach, double alpha, double beta, double delta_x) = 0;

	virtual double GetCY(double t, double mach, double alpha, double beta, double delta_x, double delta_y, double delta_z) = 0;
	virtual double GetMZ(double t, double mach, double alpha, double beta, double delta_z) = 0;
	virtual double GetMZWZ(double t, double mach, double alpha, double beta, double delta_z) = 0;

	virtual double GetCZ(double t, double mach, double alpha, double beta, double delta_x, double delta_y, double delta_z) = 0;
	virtual double GetMY(double t, double mach, double alpha, double beta, double delta_y) = 0;
	virtual double GetMYWY(double t, double mach, double alpha, double beta, double delta_y) = 0;

	//计算各种导数（为了支持动力系数计算）
	//升力系数、俯仰力矩对攻角、俯仰舵偏角的导数
	virtual double GetCY_Alpha(double t, double mach, double alpha, double beta) = 0;
	virtual double GetCY_DeltaZ(double t, double mach, double alpha, double beta, double delta_z) = 0;
	virtual double GetMZ_Alpha(double t, double mach, double alpha, double beta) = 0;
	virtual double GetMZ_DeltaZ(double t, double mach, double alpha, double beta, double delta_z) = 0;

	//侧向力系数、偏航力矩对侧滑角、偏航舵偏角的导数
	virtual double GetCZ_Beta(double t, double mach, double alpha, double beta) = 0;
	virtual double GetCZ_DeltaY(double t, double mach, double alpha, double beta, double delta_y) = 0;
	virtual double GetMY_Beta(double t, double mach, double alpha, double beta) = 0;
	virtual double GetMY_DeltaY(double t, double mach, double alpha, double beta, double delta_y) = 0;

	//滚转力矩对滚转舵偏角的导数
	virtual double GetMX_DeltaX(double t, double mach, double alpha, double beta, double delta_x) = 0;

	//马格努斯力矩对侧滑角的导数
	virtual double GetMYWX_Beta_Rad(double t, double mach, double alpha, double beta) = 0;

};

class NormalAerodynamicModel:public AerodynamicModel {
public:
	NormalAerodynamicModel(const char* aero_data_dir);
	~NormalAerodynamicModel();
	//气动力计算
	virtual void calc_aerodynamic_force(double t, double V, double sonic, double rho,
		double alpha, double beta,
		double delta_x, double delta_y, double delta_z,
		double& X, double& Y, double& Z
		) ;
	//气动力矩计算
	virtual void calc_aerodynamic_torque(double t, double V, double sonic, double rho,
		double alpha, double beta,
		double delta_x, double delta_y, double delta_z,
		double wx, double wy, double wz,
		double& Mx, double& My, double& Mz) ;


	virtual double GetS() ;
	virtual double GetL() ;
	virtual double GetL_MX() ;


	virtual double GetMaxAlpha() { //工况中的最大攻角
		return ::GetMaxAlpha(this->aero_data);
	}
	virtual double GetMinAlpha() {  //工况中的最小攻角
		return ::GetMinAlpha(this->aero_data);
	}

	virtual double GetMaxBeta() {
		//工况中的最大侧滑角
		return ::GetMaxBeta(this->aero_data);
	}
	virtual double GetMinBeta()  //工况中的最小侧滑角
	{
		return ::GetMinBeta(this->aero_data);
	}

	virtual double GetBalanceAlpha(double t, double mach, double beta, double delta_z) ;
	virtual double GetBalanceBeta(double t, double mach, double alpha, double delta_y) ;
	virtual double GetMaxCLCD_Alpha(double t, double mach, double beta) ;
	virtual double GetBalanceDeltaZ(double t, double mach, double alpha, double beta) ;
	virtual double GetBalanceDeltaY(double t, double mach, double alpha, double beta) ;
	virtual double GetCYBalanceByAlpha(double t, double mach, double alpha, double beta) ;
	virtual double GetCZBalanceByBeta(double t, double mach, double alpha, double beta) ;

	virtual double GetCX(double t, double mach, double alpha, double beta, double delta_x, double delta_y, double delta_z);
	virtual double GetMX(double t, double mach, double alpha, double beta, double delta_x);
	virtual double GetMXWX(double t, double mach, double alpha, double beta, double delta_x);

	virtual double GetCY(double t, double mach, double alpha, double beta, double delta_x, double delta_y, double delta_z);
	virtual double GetMZ(double t, double mach, double alpha, double beta, double delta_z);
	virtual double GetMZWZ(double t, double mach, double alpha, double beta, double delta_z);

	virtual double GetCZ(double t, double mach, double alpha, double beta, double delta_x, double delta_y, double delta_z);
	virtual double GetMY(double t, double mach, double alpha, double beta, double delta_y);
	virtual double GetMYWY(double t, double mach, double alpha, double beta, double delta_y);

	//计算各种导数（为了支持动力系数计算）
	//升力系数、俯仰力矩对攻角、俯仰舵偏角的导数
	virtual double GetCY_Alpha(double t, double mach, double alpha, double beta);
	virtual double GetCY_DeltaZ(double t, double mach, double alpha, double beta, double delta_z);
	virtual double GetMZ_Alpha(double t, double mach, double alpha, double beta);
	virtual double GetMZ_DeltaZ(double t, double mach, double alpha, double beta, double delta_z);

	//侧向力系数、偏航力矩对侧滑角、偏航舵偏角的导数
	virtual double GetCZ_Beta(double t, double mach, double alpha, double beta);
	virtual double GetCZ_DeltaY(double t, double mach, double alpha, double beta, double delta_y);
	virtual double GetMY_Beta(double t, double mach, double alpha, double beta);
	virtual double GetMY_DeltaY(double t, double mach, double alpha, double beta, double delta_y);

	//滚转力矩对滚转舵偏角的导数
	virtual double GetMX_DeltaX(double t, double mach, double alpha, double beta, double delta_x);

	//马格努斯力矩对侧滑角的导数
	virtual double GetMYWX_Beta_Rad(double t, double mach, double alpha, double beta);

private:
	aerodynamic_data aero_data;
	std::string _aero_data_path;
};


class ThrustAerodynamicModel : public AerodynamicModel {
public:
	double t_start_;
	//纵向质心时间序列，弹头为0点
	std::vector<double> t_array;
	std::vector<double> xc_array;

	double xc_before_;//发动机工作前的全弹质心
	double xc_after_;//发动机工作后的全弹质心

public:
	aerodynamic_data aero_data_before;//发动机工作前气动数据
	aerodynamic_data aero_data_after;//发动机工作后气动数据

private:
	double calc_x_center(double t) {
		return interp11(xc_array, t_array, t - t_start_);
	}
public:
	ThrustAerodynamicModel(double t_start) { t_start_ = t_start; };
	ThrustAerodynamicModel(double t_start, const char* aero_data_before_dir, const char* aero_data_after_dir);
	~ThrustAerodynamicModel();
	//气动力计算
	virtual void calc_aerodynamic_force(double t, double V, double sonic, double rho,
		double alpha, double beta,
		double delta_x, double delta_y, double delta_z,
		double& X, double& Y, double& Z
		);
	//气动力矩计算
	virtual void calc_aerodynamic_torque(double t, double V, double sonic, double rho,
		double alpha, double beta,
		double delta_x, double delta_y, double delta_z,
		double wx, double wy, double wz,
		double& Mx, double& My, double& Mz);


	virtual double GetS();
	virtual double GetL();
	virtual double GetL_MX() ;

	virtual double GetMaxAlpha() { //工况中的最大攻角
		return ::GetMaxAlpha(this->aero_data_before);
	}
	virtual double GetMinAlpha() {  //工况中的最小攻角
		return ::GetMinAlpha(this->aero_data_before);
	}

	virtual double GetMaxBeta() {  //工况中的最大侧滑角
		return ::GetMaxBeta(this->aero_data_before);
	}
	virtual double GetMinBeta()  //工况中的最小侧滑角
	{
		return ::GetMinBeta(this->aero_data_before);
	}


	virtual double GetBalanceAlpha(double t, double mach, double beta, double delta_z);
	virtual double GetBalanceBeta(double t, double mach, double alpha, double delta_y);
	virtual double GetMaxCLCD_Alpha(double t, double mach, double beta);
	virtual double GetBalanceDeltaZ(double t, double mach, double alpha, double beta);
	virtual double GetBalanceDeltaY(double t, double mach, double alpha, double beta);
	virtual double GetCYBalanceByAlpha(double t, double mach, double alpha, double beta);
	virtual double GetCZBalanceByBeta(double t, double mach, double alpha, double beta);

	virtual double GetCX(double t, double mach, double alpha, double beta, double delta_x, double delta_y, double delta_z);
	virtual double GetMX(double t, double mach, double alpha, double beta, double delta_x);
	virtual double GetMXWX(double t, double mach, double alpha, double beta, double delta_x);

	virtual double GetCY(double t, double mach, double alpha, double beta, double delta_x, double delta_y, double delta_z);
	virtual double GetMZ(double t, double mach, double alpha, double beta, double delta_z);
	virtual double GetMZWZ(double t, double mach, double alpha, double beta, double delta_z);

	virtual double GetCZ(double t, double mach, double alpha, double beta, double delta_x, double delta_y, double delta_z);
	virtual double GetMY(double t, double mach, double alpha, double beta, double delta_y);
	virtual double GetMYWY(double t, double mach, double alpha, double beta, double delta_y);

	//计算各种导数（为了支持动力系数计算）
	//升力系数、俯仰力矩对攻角、俯仰舵偏角的导数
	virtual double GetCY_Alpha(double t, double mach, double alpha, double beta);
	virtual double GetCY_DeltaZ(double t, double mach, double alpha, double beta, double delta_z);
	virtual double GetMZ_Alpha(double t, double mach, double alpha, double beta);
	virtual double GetMZ_DeltaZ(double t, double mach, double alpha, double beta, double delta_z);

	//侧向力系数、偏航力矩对侧滑角、偏航舵偏角的导数
	virtual double GetCZ_Beta(double t, double mach, double alpha, double beta);
	virtual double GetCZ_DeltaY(double t, double mach, double alpha, double beta, double delta_y);
	virtual double GetMY_Beta(double t, double mach, double alpha, double beta);
	virtual double GetMY_DeltaY(double t, double mach, double alpha, double beta, double delta_y);

	//滚转力矩对滚转舵偏角的导数
	virtual double GetMX_DeltaX(double t, double mach, double alpha, double beta, double delta_x);

	//马格努斯力矩对侧滑角的导数
	virtual double GetMYWX_Beta_Rad(double t, double mach, double alpha, double beta);

};
////气动力模型
//inline void aerodynamic_force_model_rad(aerodynamic_data aero_data, double V, double sonic, double rho,
//	double alpha, double beta,
//	double delta_x, double delta_y, double delta_z,
//	double& X, double& Y, double& Z
//	)
//{
//	double S = GetS(aero_data);
//	double mach = V / sonic;
//	double q = 0.5 * rho * V * V;
//
//	double CX = GetCX_Rad(aero_data, mach, alpha, beta, delta_x, delta_y, delta_z); //阻力系数
//	double CY = GetCY_Rad(aero_data, mach, alpha, beta, delta_x, delta_y, delta_z);
//	double CZ = GetCZ_Rad(aero_data, mach, alpha, beta, delta_x, delta_y, delta_z);
//	X = CX*q*S; //阻力
//	Y = CY*q*S; //升力
//	Z = CZ*q*S;//侧向力
//}
//
////气动力矩模型
//inline void aerodynamic_torque_model_rad(aerodynamic_data aero_data, double V, double sonic, double rho,
//	double alpha, double beta, 
//	double delta_x, double delta_y, double delta_z,
//	double wx, double wy, double wz,
//	double& Mx, double& My, double& Mz
//){
//	double S = GetS(aero_data);
//	double L = GetL(aero_data);
//	double mach = V / sonic;
//	double q = 0.5 * rho * V * V;
//
//	double mx = GetMX_Rad(aero_data, mach, alpha, beta, delta_x);
//	double mxwx = GetMXWX_Rad(aero_data, mach, alpha, beta, delta_x);
//	double my = GetMY_Rad(aero_data, mach, alpha, beta, delta_y);
//	double mywy = GetMYWY_Rad(aero_data, mach, alpha, beta, delta_y);
//	double mz = GetMZ_Rad(aero_data, mach, alpha, beta, delta_z);
//	double mzwz = GetMZWZ_Rad(aero_data, mach, alpha, beta, delta_z);
//
//	Mx = (mx + mxwx * wx*RAD*L / (2 * V))*q*S*L;
//	My = (my + mywy * wy*RAD*L / (2 * V))*q*S*L;
//	Mz = (mz + mzwz * wz*RAD*L / (2 * V))*q*S*L;
//
//}
//

////气动力模型
//inline void aerodynamic_force_model(aerodynamic_data aero_data, double V, double sonic, double rho,
//	double alpha_du, double beta_du,
//	double delta_x_du, double delta_y_du, double delta_z_du,
//	double& X, double& Y, double& Z
//	)
//{
//	double S = GetS(aero_data);
//	double mach = V / sonic;
//	double q = 0.5 * rho * V * V;
//
//	double CX = GetCX(aero_data, mach, alpha_du, beta_du, delta_x_du, delta_y_du, delta_z_du); //阻力系数
//	double CY = GetCY(aero_data, mach, alpha_du, beta_du, delta_x_du, delta_y_du, delta_z_du);
//	double CZ = GetCZ(aero_data, mach, alpha_du, beta_du, delta_y_du);
//	X = CX*q*S; //阻力
//	Y = CY*q*S; //升力
//	Z = CZ*q*S;//侧向力
//}
//
////气动力矩模型
//inline void aerodynamic_torque_model(aerodynamic_data aero_data, double V, double sonic, double rho,
//	double alpha_du, double beta_du, 
//	double delta_x_du, double delta_y_du, double delta_z_du,
//	double wx, double wy, double wz,
//	double& Mx, double& My, double& Mz
//){
//	double S = GetS(aero_data);
//	double L = GetL(aero_data);
//	double mach = V / sonic;
//	double q = 0.5 * rho * V * V;
//
//	double mx = GetMX(aero_data, mach, alpha_du, beta_du, delta_x_du);
//	double mxwx = GetMXWX(aero_data, mach, alpha_du, beta_du, delta_x_du);
//	double my = GetMY(aero_data, mach, alpha_du, beta_du, delta_y_du);
//	double mywy = GetMYWY(aero_data, mach, alpha_du, beta_du, delta_y_du);
//	double mz = GetMZ(aero_data, mach, alpha_du, beta_du, delta_z_du);
//	double mzwz = GetMZWZ(aero_data, mach, alpha_du, beta_du, delta_z_du);
//
//	Mx = (mx + mxwx * wx*RAD*L / (2 * V))*q*S*L;
//	My = (my + mywy * wy*RAD*L / (2 * V))*q*S*L;
//	Mz = (mz + mzwz * wz*RAD*L / (2 * V))*q*S*L;
//
//}