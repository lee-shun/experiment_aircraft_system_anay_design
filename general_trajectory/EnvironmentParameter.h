#pragma once
#include "../interp.h"
class EnvironmentParameter
{
private:
	std::vector<double> ANDH;//�߶�
	std::vector<double> ANDSonic;//����
	std::vector<double> ANDRHO; //�ܶ�
public:
	//void Load(std::string path);
	void Load();

	double GetSonic(double h){
		//���ݸ߶Ȳ�ֵ�������
		return interp11(ANDSonic, ANDH, h);
	}
	double GetRHO(double h){
		//���ݸ߶Ȳ�ֵ��������ܶ�
		return interp11(ANDRHO, ANDH, h);
	}
};

