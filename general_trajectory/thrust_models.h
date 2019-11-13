//推力模型
#pragma once

#include "stdafx.h"
#include "../interp.h"

//double inline zero_thrust(double t){
//	return 0;
//}
//

class ThrustModel {
public:
	static ThrustModel* ConstructFromFile(double t_start, const char* filename);
public:
	virtual ~ThrustModel() {};
	virtual double get_current_thrust(double t) = 0;
};

class ZeorThrustModel :public ThrustModel {
public:
	virtual double get_current_thrust(double t) {
		return 0;
	}
};

class NormalThrustModel : public ThrustModel {
private:
	double t_start_;

public:
	//推力时间序列
	std::vector<double> ts_;
	std::vector<double> thrusts_;

public:
	NormalThrustModel(double t_start) {
		t_start_ = t_start;
	}
	virtual double get_current_thrust(double t) {
		return interp11(this->thrusts_, this->ts_, t - t_start_);
	}
};

