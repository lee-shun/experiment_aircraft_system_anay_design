#pragma once
#include "const.h"
#include "trajectory.h"

class six_degree_trajectory_without_control: public trajectory
{
public:
	enum trajectory_state
	{
		V_Index,
		Theta_Index,
		Psi_V_Index,
		WX_Index,
		WY_Index,
		WZ_Index,
		X_Index,
		Y_Index,
		Z_Index,
		VarTheta_Index,
		Psi_Index,
		Gamma_Index,
		Mass_Index,
		DeltaX_Index, //��ת��ƫ��
		DeltaY_Index,
		DeltaZ_Index,
		//Delta_P
		Number_RungeKutta
	};

public:
	//�����������δ֪��������, ����Ƕ�ֵ�ĵ�λΪ���ȡ���֯��������ʽ��Ϊ�������������
	double states[Number_RungeKutta] ; 

	virtual int get_number_of_runge_kutta(){ return Number_RungeKutta; }


	virtual void no_differential_calculus_equations(double t , double dt) {}//�����ɶȵ�����û��������Ʒ��̣���˴˺�������Ϊ��

	void output_title(FILE* fout){
		fprintf(fout, "Time(s)		V,		Theta,		Psi_V,		WX,		WY,		WZ,		X,		Y,		Z,		VarTheta,		Psi,		Gamma,		Mass, DeltaX, DeltaY, DeltaZ,	Alpha,	Beta,	Gamma_V \n");
	}
	void result(FILE* fout,double y[], double t){
		fprintf(fout, "%15.10f ", t);

		//���������y
		double output_y[Number_RungeKutta];
		std::copy(y, y + Number_RungeKutta, output_y);
		//�����������yת��Ϊ�Ƕ�
		output_y[Theta_Index] = output_y[Theta_Index] * RAD;
		output_y[Psi_V_Index] = output_y[Psi_V_Index] * RAD;
		output_y[WX_Index] = output_y[WX_Index] * RAD;
		output_y[WY_Index] = output_y[WY_Index] * RAD;
		output_y[WZ_Index] = output_y[WZ_Index] * RAD;
		output_y[Psi_Index] = output_y[Psi_Index] * RAD;
		output_y[VarTheta_Index] = output_y[VarTheta_Index] * RAD;
		output_y[Gamma_Index] = output_y[Gamma_Index] * RAD;

		for (int i = 0; i < Number_RungeKutta; i++)
			fprintf(fout, "%15.10f ", output_y[i]);

		///!��Ҫ�����ǵȼ���״̬��
		//fprintf(fout, "%15.10f ", _alpha*RAD);
		//fprintf(fout, "%15.10f ", _beta*RAD);
		//fprintf(fout, "%15.10f ", _gamma_v*RAD);

		fprintf(fout, "\n");
	}
	bool end(){
		return states[Y_Index] <= 0;
	}
public:
	six_degree_trajectory_without_control();
	~six_degree_trajectory_without_control();

private:
	virtual void trajectory_equations_internal(double t, double dt, double y[], double dy_dt[]);
};

