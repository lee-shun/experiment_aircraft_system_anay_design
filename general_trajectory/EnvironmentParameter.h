#pragma once
#include "../interp.h"
class EnvironmentParameter
{
private:
	std::vector<double> ANDH;//高度
	std::vector<double> ANDSonic;//音速
	std::vector<double> ANDRHO; //密度
public:
	//void Load(std::string path);
	void Load();

	double GetSonic(double h){
		//根据高度插值求出音速
		return interp11(ANDSonic, ANDH, h);
	}
	double GetRHO(double h){
		//根据高度插值求出空气密度
		return interp11(ANDRHO, ANDH, h);
	}
};

