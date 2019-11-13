//������ת�����˶�ѧ�Ͷ���ѧ������

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
	double vartheta, //������
	//double psi, //ƫ����
	double gamma,
	double& dwx_dt,
	double& dwy_dt,
	double& dwz_dt,
	double& dvartheta_dt,
	double& dpsi_dt,
	double& dgamma_dt
	)
{
	//������ת���Ķ���ѧ
	dwx_dt = (Mx - (Jz - Jy)*wz*wy) / Jx;
	dwy_dt = (My - (Jx - Jz)*wx*wz) / Jy;
	dwz_dt = (Mz - (Jy - Jx)*wy*wx) / Jz;

	//������ת�����˶�ѧ 
	dvartheta_dt = wy*sin(gamma) + wz*cos(gamma);
	dpsi_dt = (wy*cos(gamma) - wz*sin(gamma)) / cos(vartheta);
	dgamma_dt = wx -
		tan(vartheta) * (wy*cos(gamma) - wz*sin(gamma));
}