//绕质心转动的运动学和动力学方程组

#pragma once


#include "math.h"
inline void centroid_rotate_equations(
	double Mx,
	double My,
	double Mz,
	double Jx,
	double Jy,
	double Jz,
	double wx,
	double wy,
	double wz,
	double vartheta, //俯仰角
	//double psi, //偏航角
	double gamma,
	double& dwx_dt,
	double& dwy_dt,
	double& dwz_dt,
	double& dvartheta_dt,
	double& dpsi_dt,
	double& dgamma_dt
	)
{
	//绕质心转动的动力学
	dwx_dt = (Mx - (Jz - Jy)*wz*wy) / Jx;
	dwy_dt = (My - (Jx - Jz)*wx*wz) / Jy;
	dwz_dt = (Mz - (Jy - Jx)*wy*wx) / Jz;

	//绕质心转动的运动学 
	dvartheta_dt = wy*sin(gamma) + wz*cos(gamma);
	dpsi_dt = (wy*cos(gamma) - wz*sin(gamma)) / cos(vartheta);
	dgamma_dt = wx -
		tan(vartheta) * (wy*cos(gamma) - wz*sin(gamma));
}