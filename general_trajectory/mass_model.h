//质量模型
#pragma once
#include "stdafx.h"
#include "..\interp.h"




class MassFlowRateModel{
public:
	static MassFlowRateModel* ConstructFromFile(double t_start, const char* filename);
	virtual double mass_flow_rate_equations(double t) = 0;  //计算dm/dt
	virtual ~MassFlowRateModel(){}
};

//固定质量秒流量变化的质量模型（包含无质量变化的质量模型）
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

//正常质量秒流量模型
class NormalMassFlowRateModel: public MassFlowRateModel {
private:
	double t_start_;

public:
	//质量秒流量时间序列
	std::vector<double> ts_;
	std::vector<double> mcs_;

public:
	NormalMassFlowRateModel(double t_start) {
		t_start_ = t_start;
	}
	virtual double mass_flow_rate_equations(double t) {
		//质量秒流量， 对应质量方程中质量的减少，注意取负号
		return -interp11(this->mcs_, this->ts_, t - t_start_);
	}

	void Load(std::map<std::string, std::string> n2vs) ;
};

//提供的输入序列时质量随时间变化的曲线，需要在内部转换为质量秒流量序列
class NormalMassModel : public MassFlowRateModel {
private:
	double t_start_;

public:
	//质量时间序列
	std::vector<double> ts_;
	std::vector<double> ms_;


	NormalMassModel(double t_start) {
		t_start_ = t_start;
	}

	void Load(std::map<std::string, std::string> n2vs) ;

	virtual double mass_flow_rate_equations(double t) {
		//质量秒流量， 对应质量方程中质量的减少，注意取负号
		if (ts_.size() == 1)
			return 0; //只有一个值，假定质量不变。

		double t_elapsed = t-t_start_;

		if (t_elapsed < *ts_.begin() ||  t_elapsed >= *ts_.rbegin()) //如果大于等于时间序列中的最后一个时间，认为质量秒流量为0
			return 0;
		
		//找到一个i，使得 t[i] <= t_elapsed < t[i+1]
		int i = 0;
		while (t_elapsed >= ts_[i + 1] )
			i++;

		//在[t[i] , t[i+1])区间内的质量秒流量，等于质量差除以时间区间的长度
		return  (ms_[i + 1] - ms_[i]) / (ts_[i + 1] - ts_[i]);
	}
};