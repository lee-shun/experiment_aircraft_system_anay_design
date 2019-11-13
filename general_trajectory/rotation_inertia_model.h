#pragma once
#include "stdafx.h"
#include "..\interp.h"
#include "utils.h"

//转动惯量模型
class RotationInertiaModel {
public:
	virtual ~RotationInertiaModel() {};
	virtual double GetJX(double t) = 0;
	virtual double GetJY(double t) = 0;
	virtual double GetJZ(double t) = 0;

	static RotationInertiaModel* ConstructRotationInertiaModelFromFile(double t_start, std::string filename);
};

class FixedRotationInertiaModel : public RotationInertiaModel {
public:
	double JX_;
	double JY_;
	double JZ_;

public:
	virtual double GetJX(double t) {
		return JX_;
	}
	virtual double GetJY(double t) {
		return JY_;
	}
	virtual double GetJZ(double t) {
		return JZ_;
	}
	
};


void LoadFixedRotationInertiaModel(FixedRotationInertiaModel* model, Name2Value& n2vs) ;



class GeneralRotationInertiaModel : public RotationInertiaModel {
private:
	double t_start_;
public:
	std::vector<double> ts_;
	std::vector<double> JXs_;
	std::vector<double> JYs_;
	std::vector<double> JZs_;

public:
	GeneralRotationInertiaModel(double t_start) {
		t_start_ = t_start;
	}
	virtual double GetJX(double t) {
		return interp11(this->JXs_, this->ts_, t - t_start_);
	}
	virtual double GetJY(double t) {
		return interp11(this->JYs_, this->ts_, t - t_start_);
	}
	virtual double GetJZ(double t) {
		return interp11(this->JZs_, this->ts_, t - t_start_);
	}
};

void LoadGeneralRotationInertiaModel(GeneralRotationInertiaModel* model,  Name2Value& n2vs) ;
