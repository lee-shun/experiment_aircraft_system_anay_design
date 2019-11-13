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
		DeltaX_Index, //滚转舵偏角
		DeltaY_Index,
		DeltaZ_Index,
		//Delta_P
		Number_RungeKutta
	};

public:
	//弹道方程组的未知量的数组, 里面角度值的单位为弧度。组织成数组形式是为了龙格库塔积分
	double states[Number_RungeKutta] ; 

	virtual int get_number_of_runge_kutta(){ return Number_RungeKutta; }


	virtual void no_differential_calculus_equations(double t , double dt) {}//六自由度弹道，没有理想控制方程，因此此函数保持为空

	void output_title(FILE* fout){
		fprintf(fout, "Time(s)		V,		Theta,		Psi_V,		WX,		WY,		WZ,		X,		Y,		Z,		VarTheta,		Psi,		Gamma,		Mass, DeltaX, DeltaY, DeltaZ,	Alpha,	Beta,	Gamma_V \n");
	}
	void result(FILE* fout,double y[], double t){
		fprintf(fout, "%15.10f ", t);

		//用于输出的y
		double output_y[Number_RungeKutta];
		std::copy(y, y + Number_RungeKutta, output_y);
		//将用于输出的y转换为角度
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

		///!需要将攻角等加入状态量
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

