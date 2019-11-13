//角度几何关系方程
#pragma once
#include <math.h>

inline void angle_geometry_equations(double theta, double psi_v, double vartheta, double psi, double gamma,
	double &alpha, double& beta, double& gamma_v
	)
{
	beta = asin(
		cos(theta)
		* (cos(gamma)*sin(psi - psi_v) + sin(vartheta)*sin(gamma)*cos(psi - psi_v))
		- sin(theta)*cos(vartheta)*sin(gamma));
	alpha = asin(
		(
		cos(theta)
		*(sin(vartheta)*cos(psi - psi_v) - sin(gamma)*sin(psi - psi_v))
		- sin(theta)*cos(vartheta)*cos(gamma)
		) / cos(beta)
		);
	gamma_v = asin(
		(
		cos(alpha)*sin(beta)*sin(vartheta)
		- sin(alpha)*sin(beta)*cos(gamma)*cos(vartheta)
		+ cos(beta)*sin(gamma)*cos(vartheta)
		) / cos(theta)
		);
}