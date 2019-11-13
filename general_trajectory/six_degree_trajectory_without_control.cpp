#include "stdafx.h"
#include "six_degree_trajectory_without_control.h"
#include "angle_geometry_equations.h"
#include "thrust_models.h"
#include "aerodynamic_model.h"
#include "CentroidMotionEquations.h"
#include "CentroidRotateEquations.h"
#include "mass_model.h"
#include "ControlEquations.h"

six_degree_trajectory_without_control::six_degree_trajectory_without_control() //:states{0}
{
	memset(states, 0, Number_RungeKutta*sizeof(double));
}


six_degree_trajectory_without_control::~six_degree_trajectory_without_control()
{
}

void six_degree_trajectory_without_control::trajectory_equations_internal(double t, double dt, double y[], double dy_dt[]){
#if 0
	//对应上述方程组变量的引用
	double& V = y[V_Index]; //速度
	double& theta = y[Theta_Index]; //弹道倾角
	double& psi_v = y[Psi_V_Index]; //弹道偏角
	double& wx = y[WX_Index];
	double& wy = y[WY_Index];
	double& wz = y[WZ_Index];
	double& x = y[X_Index];
	double& y_ = y[Y_Index]; //为了避免名称冲突，命名成y_
	double& z = y[Z_Index];
	double& vartheta = y[VarTheta_Index];
	double& psi = y[Psi_Index];
	double& gamma = y[Gamma_Index];
	double& m = y[Mass_Index];
	double& delta_x = y[DeltaX_Index];
	double& delta_y = y[DeltaY_Index];
	double& delta_z = y[DeltaZ_Index];


	//角度模型
	angle_geometry_equations(theta, psi_v, vartheta, psi, gamma, alpha, beta, gamma_v);

	//推力模型, 采用0推力模型
	double P = zero_thrust(t);

	//气动力和力矩计算
	//double alpha_du = alpha * RAD;
	//double beta_du = beta * RAD;
	//double delta_x_du = delta_x * RAD;
	//double delta_y_du = delta_y * RAD;
	//double delta_z_du = delta_z * RAD;
	double sonic = g_atmosphere_model.GetSonic(y_);
	double rho = g_atmosphere_model.GetRHO(y_);
	//气动力模型
	double X, Y, Z;
	//aerodynamic_force_model(aero_data, V, sonic, rho, alpha_du, beta_du, delta_x_du, delta_y_du, delta_z_du, X, Y, Z);
	aerodynamic_force_model_rad(aero_data, V, sonic, rho, alpha, beta, delta_x, delta_y, delta_z, X, Y, Z);
	//气动力矩模型
	double Mx, My, Mz;
	//aerodynamic_torque_model(aero_data, V, sonic, rho,
	//	alpha_du, beta_du,
	//	delta_x_du, delta_y_du, delta_z_du,
	//	wx, wy, wz, Mx, My, Mz);
	aerodynamic_torque_model_rad(aero_data, V, sonic, rho,
		alpha, beta,
		delta_x, delta_y, delta_z,
		wx, wy, wz, Mx, My, Mz);


	//质心运动模型
	centroid_motion_equations(P, alpha, beta, gamma_v, X, Y, Z, V, theta,
		psi_v, x, y_, z, m, dy_dt[V_Index], dy_dt[Theta_Index], dy_dt[Psi_V_Index], dy_dt[X_Index],
		dy_dt[Y_Index], dy_dt[Z_Index]);

	//质心转动模型
	centroid_rotate_equations(Mx, My, Mz, Jx, Jy, Jz, wx, wy, wz, 
		vartheta, gamma, 
		dy_dt[WX_Index], dy_dt[WY_Index], dy_dt[WZ_Index], 
		dy_dt[VarTheta_Index], dy_dt[Psi_Index], dy_dt[Gamma_Index]);

	//质量模型
	dy_dt[Mass_Index] = zero_mass_equations(t);

	//控制模型
	no_control(dy_dt[DeltaX_Index], dy_dt[DeltaY_Index], dy_dt[DeltaZ_Index]);
#endif
}
