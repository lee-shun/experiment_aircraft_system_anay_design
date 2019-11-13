//�����������ؼ���ģ��
#pragma once
//#include "..\AeroInterp\aerodynamic_interp\aerodynamic_interp.h"
#include "stdafx.h"
#include "my_aerodynamic_interp.h"
#include "..\interp.h"

class AerodynamicModel {
public:
	static AerodynamicModel* ConstructFromFile(double t_start, const char* filename);

	//����������
	virtual void calc_aerodynamic_force(double t, double V, double sonic, double rho,
		double alpha, double beta,
		double delta_x, double delta_y, double delta_z,
		double& X, double& Y, double& Z
		) = 0;
	//�������ؼ���
	virtual void calc_aerodynamic_torque(double t, double V, double sonic, double rho,
		double alpha, double beta,
		double delta_x, double delta_y, double delta_z,
		double wx, double wy, double wz,
		double& Mx, double& My, double& Mz) = 0;

	virtual double GetS() = 0;
	virtual double GetL() = 0;
	virtual double GetL_MX() = 0;


	virtual double GetMaxAlpha() = 0; //�����е���󹥽�
	virtual double GetMinAlpha() = 0; //�����е���С����

	virtual double GetMaxBeta() = 0; //�����е����໬��
	virtual double GetMinBeta() = 0; //�����е���С�໬��

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

	//������ֵ�����Ϊ��֧�ֶ���ϵ�����㣩
	//����ϵ�����������ضԹ��ǡ�������ƫ�ǵĵ���
	virtual double GetCY_Alpha(double t, double mach, double alpha, double beta) = 0;
	virtual double GetCY_DeltaZ(double t, double mach, double alpha, double beta, double delta_z) = 0;
	virtual double GetMZ_Alpha(double t, double mach, double alpha, double beta) = 0;
	virtual double GetMZ_DeltaZ(double t, double mach, double alpha, double beta, double delta_z) = 0;

	//������ϵ����ƫ�����ضԲ໬�ǡ�ƫ����ƫ�ǵĵ���
	virtual double GetCZ_Beta(double t, double mach, double alpha, double beta) = 0;
	virtual double GetCZ_DeltaY(double t, double mach, double alpha, double beta, double delta_y) = 0;
	virtual double GetMY_Beta(double t, double mach, double alpha, double beta) = 0;
	virtual double GetMY_DeltaY(double t, double mach, double alpha, double beta, double delta_y) = 0;

	//��ת���ضԹ�ת��ƫ�ǵĵ���
	virtual double GetMX_DeltaX(double t, double mach, double alpha, double beta, double delta_x) = 0;

	//���Ŭ˹���ضԲ໬�ǵĵ���
	virtual double GetMYWX_Beta_Rad(double t, double mach, double alpha, double beta) = 0;

};

class NormalAerodynamicModel:public AerodynamicModel {
public:
	NormalAerodynamicModel(const char* aero_data_dir);
	~NormalAerodynamicModel();
	//����������
	virtual void calc_aerodynamic_force(double t, double V, double sonic, double rho,
		double alpha, double beta,
		double delta_x, double delta_y, double delta_z,
		double& X, double& Y, double& Z
		) ;
	//�������ؼ���
	virtual void calc_aerodynamic_torque(double t, double V, double sonic, double rho,
		double alpha, double beta,
		double delta_x, double delta_y, double delta_z,
		double wx, double wy, double wz,
		double& Mx, double& My, double& Mz) ;


	virtual double GetS() ;
	virtual double GetL() ;
	virtual double GetL_MX() ;


	virtual double GetMaxAlpha() { //�����е���󹥽�
		return ::GetMaxAlpha(this->aero_data);
	}
	virtual double GetMinAlpha() {  //�����е���С����
		return ::GetMinAlpha(this->aero_data);
	}

	virtual double GetMaxBeta() {
		//�����е����໬��
		return ::GetMaxBeta(this->aero_data);
	}
	virtual double GetMinBeta()  //�����е���С�໬��
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

	//������ֵ�����Ϊ��֧�ֶ���ϵ�����㣩
	//����ϵ�����������ضԹ��ǡ�������ƫ�ǵĵ���
	virtual double GetCY_Alpha(double t, double mach, double alpha, double beta);
	virtual double GetCY_DeltaZ(double t, double mach, double alpha, double beta, double delta_z);
	virtual double GetMZ_Alpha(double t, double mach, double alpha, double beta);
	virtual double GetMZ_DeltaZ(double t, double mach, double alpha, double beta, double delta_z);

	//������ϵ����ƫ�����ضԲ໬�ǡ�ƫ����ƫ�ǵĵ���
	virtual double GetCZ_Beta(double t, double mach, double alpha, double beta);
	virtual double GetCZ_DeltaY(double t, double mach, double alpha, double beta, double delta_y);
	virtual double GetMY_Beta(double t, double mach, double alpha, double beta);
	virtual double GetMY_DeltaY(double t, double mach, double alpha, double beta, double delta_y);

	//��ת���ضԹ�ת��ƫ�ǵĵ���
	virtual double GetMX_DeltaX(double t, double mach, double alpha, double beta, double delta_x);

	//���Ŭ˹���ضԲ໬�ǵĵ���
	virtual double GetMYWX_Beta_Rad(double t, double mach, double alpha, double beta);

private:
	aerodynamic_data aero_data;
	std::string _aero_data_path;
};


class ThrustAerodynamicModel : public AerodynamicModel {
public:
	double t_start_;
	//��������ʱ�����У���ͷΪ0��
	std::vector<double> t_array;
	std::vector<double> xc_array;

	double xc_before_;//����������ǰ��ȫ������
	double xc_after_;//�������������ȫ������

public:
	aerodynamic_data aero_data_before;//����������ǰ��������
	aerodynamic_data aero_data_after;//��������������������

private:
	double calc_x_center(double t) {
		return interp11(xc_array, t_array, t - t_start_);
	}
public:
	ThrustAerodynamicModel(double t_start) { t_start_ = t_start; };
	ThrustAerodynamicModel(double t_start, const char* aero_data_before_dir, const char* aero_data_after_dir);
	~ThrustAerodynamicModel();
	//����������
	virtual void calc_aerodynamic_force(double t, double V, double sonic, double rho,
		double alpha, double beta,
		double delta_x, double delta_y, double delta_z,
		double& X, double& Y, double& Z
		);
	//�������ؼ���
	virtual void calc_aerodynamic_torque(double t, double V, double sonic, double rho,
		double alpha, double beta,
		double delta_x, double delta_y, double delta_z,
		double wx, double wy, double wz,
		double& Mx, double& My, double& Mz);


	virtual double GetS();
	virtual double GetL();
	virtual double GetL_MX() ;

	virtual double GetMaxAlpha() { //�����е���󹥽�
		return ::GetMaxAlpha(this->aero_data_before);
	}
	virtual double GetMinAlpha() {  //�����е���С����
		return ::GetMinAlpha(this->aero_data_before);
	}

	virtual double GetMaxBeta() {  //�����е����໬��
		return ::GetMaxBeta(this->aero_data_before);
	}
	virtual double GetMinBeta()  //�����е���С�໬��
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

	//������ֵ�����Ϊ��֧�ֶ���ϵ�����㣩
	//����ϵ�����������ضԹ��ǡ�������ƫ�ǵĵ���
	virtual double GetCY_Alpha(double t, double mach, double alpha, double beta);
	virtual double GetCY_DeltaZ(double t, double mach, double alpha, double beta, double delta_z);
	virtual double GetMZ_Alpha(double t, double mach, double alpha, double beta);
	virtual double GetMZ_DeltaZ(double t, double mach, double alpha, double beta, double delta_z);

	//������ϵ����ƫ�����ضԲ໬�ǡ�ƫ����ƫ�ǵĵ���
	virtual double GetCZ_Beta(double t, double mach, double alpha, double beta);
	virtual double GetCZ_DeltaY(double t, double mach, double alpha, double beta, double delta_y);
	virtual double GetMY_Beta(double t, double mach, double alpha, double beta);
	virtual double GetMY_DeltaY(double t, double mach, double alpha, double beta, double delta_y);

	//��ת���ضԹ�ת��ƫ�ǵĵ���
	virtual double GetMX_DeltaX(double t, double mach, double alpha, double beta, double delta_x);

	//���Ŭ˹���ضԲ໬�ǵĵ���
	virtual double GetMYWX_Beta_Rad(double t, double mach, double alpha, double beta);

};
////������ģ��
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
//	double CX = GetCX_Rad(aero_data, mach, alpha, beta, delta_x, delta_y, delta_z); //����ϵ��
//	double CY = GetCY_Rad(aero_data, mach, alpha, beta, delta_x, delta_y, delta_z);
//	double CZ = GetCZ_Rad(aero_data, mach, alpha, beta, delta_x, delta_y, delta_z);
//	X = CX*q*S; //����
//	Y = CY*q*S; //����
//	Z = CZ*q*S;//������
//}
//
////��������ģ��
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

////������ģ��
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
//	double CX = GetCX(aero_data, mach, alpha_du, beta_du, delta_x_du, delta_y_du, delta_z_du); //����ϵ��
//	double CY = GetCY(aero_data, mach, alpha_du, beta_du, delta_x_du, delta_y_du, delta_z_du);
//	double CZ = GetCZ(aero_data, mach, alpha_du, beta_du, delta_y_du);
//	X = CX*q*S; //����
//	Y = CY*q*S; //����
//	Z = CZ*q*S;//������
//}
//
////��������ģ��
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