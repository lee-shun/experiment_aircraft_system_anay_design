#pragma once
#include "stdafx.h"

//class TargetModel
//{
//public:
//	TargetModel() {};
//	virtual ~TargetModel() {};
//
//public:
//	virtual  double X(double t) = 0;
//	virtual  double Y(double t) = 0;
//	virtual  double Z(double t) = 0;
//};
//
//
//class V_TargetModel : TargetModel {
//public:
//	virtual ~V_TargetModel() {};
//
//public:
//	//目标位置
//	double x_0;
//	double y_0;
//	double z_0;
//
//	//目标运动规律
//	double v = 0;
//	double psi_v;
//	double theta;
//
//
//public:
//	virtual  double X(double t) {
//		return  x_0 + v * cos(theta)*cos(psi_v) * t ;
//	}
//	virtual  double Y(double t) {
//		return y_0 + v * sin(theta) * t;
//	}
//	virtual  double Z(double t) {
//		return z_0 - v*cos(theta)*sin(psi_v)*t;
//	}
//};