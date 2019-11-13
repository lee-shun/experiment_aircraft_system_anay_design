//��������ģ��
#pragma once


#include "const.h"
#include <math.h>
//�����˶�������
inline void centroid_motion_equations(
	double P,
	double alpha,
	double beta,
	double gamma_v,  //�������д��Yv
	double X,
	double Y,
	double Z,
	double V,
	double theta,
	double psi_v,
	double x,
	double y,
	double z,
	double m,
	double& dv_dt,
	double& dtheta_dt,
	double& dpsiv_dt, //�ٶȹ�ת��
	double& dx_dt,
	double& dy_dt,
	double& dz_dt
	)
{
	//���Ķ���ѧ
	dv_dt = (P*cos(alpha)*cos(beta) - X - m * G * sin(theta)) / m;
	dtheta_dt = (
		P * (sin(alpha)*cos(gamma_v) + cos(alpha)*sin(beta)*sin(gamma_v))
		+ Y*cos(gamma_v) - Z*sin(gamma_v) - m*G*cos(theta)
		) / (m * V);
	dpsiv_dt = (
		P * (sin(alpha)*sin(gamma_v) - cos(alpha)*sin(beta)*cos(gamma_v))
		+ Y*sin(gamma_v) + Z*cos(gamma_v)
		) / (-m*V*cos(theta));

	//�����˶�ѧ
	dx_dt = V*cos(theta)*cos(psi_v);
	dy_dt = V*sin(theta);
	dz_dt = -V*cos(theta)*sin(psi_v);

}

