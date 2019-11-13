//����ģ��
#pragma once
#include "stdafx.h"
#include "..\interp.h"




class MassFlowRateModel{
public:
	static MassFlowRateModel* ConstructFromFile(double t_start, const char* filename);
	virtual double mass_flow_rate_equations(double t) = 0;  //����dm/dt
	virtual ~MassFlowRateModel(){}
};

//�̶������������仯������ģ�ͣ������������仯������ģ�ͣ�
class FixedMassFlowRateModel:public MassFlowRateModel{
private:
	double _mc = 0;
public:
	FixedMassFlowRateModel(double mc){
		_mc = mc;
	}
	virtual double mass_flow_rate_equations(double t) {
		return _mc;
	}
};

//��������������ģ��
class NormalMassFlowRateModel: public MassFlowRateModel {
private:
	double t_start_;

public:
	//����������ʱ������
	std::vector<double> ts_;
	std::vector<double> mcs_;

public:
	NormalMassFlowRateModel(double t_start) {
		t_start_ = t_start;
	}
	virtual double mass_flow_rate_equations(double t) {
		//������������ ��Ӧ���������������ļ��٣�ע��ȡ����
		return -interp11(this->mcs_, this->ts_, t - t_start_);
	}

	void Load(std::map<std::string, std::string> n2vs) ;
};

//�ṩ����������ʱ������ʱ��仯�����ߣ���Ҫ���ڲ�ת��Ϊ��������������
class NormalMassModel : public MassFlowRateModel {
private:
	double t_start_;

public:
	//����ʱ������
	std::vector<double> ts_;
	std::vector<double> ms_;


	NormalMassModel(double t_start) {
		t_start_ = t_start;
	}

	void Load(std::map<std::string, std::string> n2vs) ;

	virtual double mass_flow_rate_equations(double t) {
		//������������ ��Ӧ���������������ļ��٣�ע��ȡ����
		if (ts_.size() == 1)
			return 0; //ֻ��һ��ֵ���ٶ��������䡣

		double t_elapsed = t-t_start_;

		if (t_elapsed < *ts_.begin() ||  t_elapsed >= *ts_.rbegin()) //������ڵ���ʱ�������е����һ��ʱ�䣬��Ϊ����������Ϊ0
			return 0;
		
		//�ҵ�һ��i��ʹ�� t[i] <= t_elapsed < t[i+1]
		int i = 0;
		while (t_elapsed >= ts_[i + 1] )
			i++;

		//��[t[i] , t[i+1])�����ڵ��������������������������ʱ������ĳ���
		return  (ms_[i + 1] - ms_[i]) / (ts_[i + 1] - ts_[i]);
	}
};