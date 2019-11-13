//理想控制模型，对应三自由度质点弹道
//根据当前状态y[]，按照理想控制规律（控制系统无误差、无延迟），计算并修正对应的状态（gamma_v, alpha, beta, delta_z, delta_y, 以及 theta,y等）
//为了通用以及方便起见， 将平衡关系式也放在了理想控制模型中，一并
#pragma once
class three_degree_trajectory;

#include "stdafx.h"
#include "../interp.h"
#include "const.h"

class IdealControlModel {
protected:
double t_start_; //起控时间
public:
	~IdealControlModel() {};

	virtual void control_equations(double t, double dt, double y[], double* delta_y, double* delta_z, double* alpha, double*beta, double* gamma_v, three_degree_trajectory* tj) = 0;

	//弹道是否可以结束（例如导弹已经命中目标），方案导引时一般不判断导弹是否结束
	virtual bool is_trajectory_end(double t, double y[]) {
		return false;
	}

	static IdealControlModel* ConstructIdealModelFromFile(double t, std::string filename, three_degree_trajectory* tj);

};
inline void ltrim(std::string &s) {
	s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int ch) {
		return !std::isspace(ch);
	}));
}

class MovementLaw {
public:
	std::vector<double> t_s;
	std::vector<double> x_s;
	std::vector<double> y_s;
	std::vector<double> z_s;

	double platform_altitude_; //平台海拔高度（作为整个发射坐标系的相对海拔）


	bool Load(const char* filename, three_degree_trajectory* tj) ;

	double X(double t) {
		return interp11(x_s, t_s, t);
	}
	double Y(double t) {
		return interp11(y_s, t_s, t) + platform_altitude_;
	}
	double Z(double t) {
		return interp11(z_s, t_s, t);
	}

	double Vx(double t) {
		double delta_t = 0.001;
		return (X(t + delta_t) - X(t)) / delta_t;
	}

	double Vy(double t) {
		double delta_t = 0.001;
		return (Y(t + delta_t) - Y(t)) / delta_t;
	}

	double Vz(double t) {
		double delta_t = 0.001;
		return (Z(t + delta_t) - Z(t)) / delta_t;
	}
};

//舵偏角控制模型（去掉了无控模型，改成了舵偏角控制模型)
class DeltaYZ_ControlModel :public IdealControlModel {
public:
	DeltaYZ_ControlModel(double t_start) {
		t_start_ = t_start;
	}

	//俯仰舵偏角时间序列
	std::vector<double> delta_z_t_;
	std::vector<double> delta_z_;

	//偏航舵偏角时间序列
	std::vector<double> delta_y_t_;
	std::vector<double> delta_y_;
public:
	void Load(Name2Value& n2vs);
public:
	virtual void control_equations(double t, double dt, double y[], double* delta_y, double* delta_z, double* alpha, double*beta, double* gamma_v, three_degree_trajectory* tj) ;
};

class BestCL_CD_ControlModel :public IdealControlModel {
public:
	BestCL_CD_ControlModel(double t_start) {
		t_start_ = t_start;
	}

public:
	void Load(Name2Value& n2vs) {};
public:
	virtual void control_equations(double t, double dt, double y[], double* delta_y, double* delta_z, double* alpha, double*beta, double* gamma_v, three_degree_trajectory* tj) ;

};


class AlphaBetaGammaV_ControlModel :public IdealControlModel {
public:
	AlphaBetaGammaV_ControlModel(double t_start) {
		t_start_ = t_start;
	}



	//攻角时间序列
	std::vector<double> alpha_t_array;
	std::vector<double> alpha_array;

	//侧滑角时间序列
	std::vector<double> beta_t_array;
	std::vector<double> beta_array;

	//侧滑角时间序列
	std::vector<double> gammav_t_array;
	std::vector<double> gammav_array;

public:
	void Load(Name2Value& n2vs);
public:
	virtual void control_equations(double t, double dt, double y[], double* delta_y, double* delta_z, double* alpha, double*beta, double* gamma_v, three_degree_trajectory* tj) ;

};

//弹道倾角/偏角理想控制模型
class ThetaPsiV_ControlModel :public IdealControlModel {
public:
	ThetaPsiV_ControlModel(double t_start) {
		t_start_ = t_start;
	}

	//倾角时间序列
	std::vector<double> theta_t_array;
	std::vector<double> theta_array;

	//偏角时间序列
	std::vector<double> psi_v_t_array;
	std::vector<double> psi_v_array;

void Load(Name2Value& n2vs);
protected:
	double Theta(double t);
	double Psi_V(double t);
public:
	virtual void control_equations(double t, double dt, double y[], double* delta_y, double* delta_z, double* alpha, double*beta, double* gamma_v, three_degree_trajectory* tj) ;

};

//俯仰角/偏航角理想控制模型
class VarTheta_Psi_ControlModel :public IdealControlModel {
public:
	VarTheta_Psi_ControlModel(double t_start) {
		t_start_ = t_start;
	}

	//俯仰角时间序列
	std::vector<double> vartheta_t_array;
	std::vector<double> vartheta_array;

	//偏角时间序列
	std::vector<double> psi_t_array;
	std::vector<double> psi_array;

void Load(Name2Value& n2vs);
protected:
	double VarTheta(double t);
	double Psi(double t);
public:
	virtual void control_equations(double t, double dt, double y[], double* delta_y, double* delta_z, double* alpha, double*beta, double* gamma_v, three_degree_trajectory* tj) ;

};



//过载控制模型
class Ny2Nz2_ControlModel :public IdealControlModel {
public:
	Ny2Nz2_ControlModel(double t_start) {
		t_start_ = t_start;
	}

	//俯仰过载时间序列
	std::vector<double> ny2_t_array;
	std::vector<double> ny2_array;

	//偏角时间序列
	std::vector<double> nz2_t_array;
	std::vector<double> nz2_array;

	void Load(Name2Value& n2vs);
protected:
	double Ny2(double t);
	double Nz2(double t);
public:
	virtual void control_equations(double t, double dt, double y[], double* delta_y, double* delta_z, double* alpha, double*beta, double* gamma_v, three_degree_trajectory* tj);

};


//纵向/侧向质心理想控制模型 （侧向质心暂时也是没有使用的？）
class YZ_ControlModel :public IdealControlModel {
public:
	YZ_ControlModel(double t_start) {
		t_start_ = t_start;
	}

	//纵向质心时间序列
	std::vector<double> y_t_array;
	std::vector<double> y_array;

	//侧向质心时间序列
	std::vector<double> z_t_array;
	std::vector<double> z_array;

void Load(Name2Value& n2vs);
protected:
	double H(double t);
	double Z(double t);
public:
	virtual void control_equations(double t, double dt, double y[], double* delta_y, double* delta_z, double* alpha, double*beta, double* gamma_v, three_degree_trajectory* tj) ;

};

//XYZ质心控制模型
class XYZ_ControlModel :public IdealControlModel {
public:
	XYZ_ControlModel(double t_start) {
		t_start_ = t_start;
	}


	//质心序列
	std::vector<double> x_array;
	std::vector<double> y_array;
	std::vector<double> z_array;

	void Load(Name2Value& n2vs);
protected:
	double H(double X_input);
	double Z(double X_input);
public:
	virtual void control_equations(double t, double dt, double y[], double* delta_y, double* delta_z, double* alpha, double*beta, double* gamma_v, three_degree_trajectory* tj);

};


//（航弹）比例导引控制模型
class Proportional_Navigation_ControlModel :public IdealControlModel {
public:
	Proportional_Navigation_ControlModel(double t_start, double K,  double x_target, double y_target = 0, double z_target = 0, double v_target_x = 0, double v_target_z = 0) {
		this->t_start_ = t_start;
		this->K = K;

	}
	Proportional_Navigation_ControlModel(double t_start) {
		this->t_start_ = t_start;
	}


	void Load(Name2Value& n2vs, three_degree_trajectory* tj);
	//比例导引系数
	double K;


	MovementLaw target_movement_;



	double distance_of_stop_nav = 0; //末端停止比例导引的弹目距离？

	double distance_of_end = 1.5;  //停止计算的弹目距离， 弹道结束条件

public:

	virtual void control_equations(double t, double dt, double y[], double* delta_y, double* delta_z, double* alpha, double*beta, double* gamma_v, three_degree_trajectory* tj) ;

	//弹道结束条件
	virtual bool is_trajectory_end(double t, double states[]);

protected:


	double calc_r(double t, double states[]) ;

	//计算纵向(vertical)平面（弹道坐标系）内的dq/dt
	double calc_dq_v_dt(double dx, double dy, double sigma, double sigma_t, double V, double V_t, double& q);
	//计算水平面（horizontal)内的dq/dt
	double calc_dq_h_dt(double dx, double dz, double sigma, double sigma_t, double V, double V_t, double& q_h, double& eta_h, double& r_h) ;
};


class Threepoint_Navigation_ControlModel :public IdealControlModel {
private:
	MovementLaw guidance_station_movement_;
	MovementLaw target_movement_;
public:

	Threepoint_Navigation_ControlModel(double t_start) {
		this->t_start_ = t_start;



		//distance_of_stop_nav = 0; //末端停止导引的弹目距离？

		distance_of_end = 0.6;  //停止计算的弹目距离， 弹道结束条件
	}



	//double distance_of_stop_nav; //末端停止比例导引的弹目距离？

	double distance_of_end;  //停止计算的弹目距离， 弹道结束条件


	//double epsilon;

public:

	void Load(Name2Value& n2vs, three_degree_trajectory* tj);

	virtual void control_equations(double t, double dt, double y[], double* delta_y, double* delta_z, double* alpha, double*beta, double* gamma_v, three_degree_trajectory* tj);

	//弹道结束条件
	virtual bool is_trajectory_end(double t, double states[]);

protected:


	double calc_r(double t, double states[]);

	double calc_depsilon_dt(double dx, double dy, double theta, double theta_t, double V, double V_t);

};


class Qianzhi_Navigation_ControlModel :public IdealControlModel {
private:
	MovementLaw guidance_station_movement_;
	MovementLaw target_movement_;
public:

	Qianzhi_Navigation_ControlModel(double t_start) {
		this->t_start_ = t_start;

		//目标起始位置，t为target缩写
		//y_t_0 = 0;
		////x_t_0 = 12000;
		//x_t_0 = 0;
		//z_t_0 = 0;

		//制导站起始位置，c为control缩写
		//y_c_0 = 12000;
		//y_c_0 = 0;
		//x_c_0 = 0;
		//z_c_0 = 0;

		////目标运动速度，对于航弹来说，假定目标只在水平面运动
		//v_t_x = 0;
		//v_t_z = 0;

		////制导站运动速度，
		//v_c_x = 0;
		//v_c_y = 0;
		//v_c_z = 0;

//		distance_of_stop_nav = 0; //末端停止比例导引的弹目距离？

		distance_of_end = 0.6;  //停止计算的弹目距离， 弹道结束条件
	}



	//目标起始位置，t为target缩写
	//double y_t_0;
	//double x_t_0;
	//double z_t_0;
	//制导站起始位置，c为control缩写
	//double y_c_0;
	//double x_c_0;
	//double z_c_0;

	//目标运动速度，对于航弹来说，假定目标只在水平面运动
	//double v_t_x;
	//double v_t_z;
	//制导站运动速度，
	//double v_c_x;
	//double v_c_y;
	//double v_c_z;

	//水平前置角
	double d_epsilon0_h = 0;// / RAD;
	//纵向前置角
	double d_epsilon0_v = 0;// 10 / RAD;

//	double distance_of_stop_nav; //末端停止比例导引的弹目距离？

	double distance_of_end;  //停止计算的弹目距离， 弹道结束条件

	//double epsilon;

public:
	void Load(Name2Value& n2vs, three_degree_trajectory* tj);

	virtual void control_equations(double t, double dt, double y[], double* delta_y, double* delta_z, double* alpha, double*beta, double* gamma_v, three_degree_trajectory* tj);

	//弹道结束条件
	virtual bool is_trajectory_end(double t, double states[]);

protected:
	//inline double x_t(double t) {
	//	return x_t_0 + v_t_x*(t - t_start_);
	//}
	//inline double y_t(double t) {
	//	return y_t_0;
	//}
	//inline double z_t(double t) {
	//	return z_t_0 + v_t_z*(t - t_start_);
	//}

	double calc_r(double t, double states[]);


};





//高飞三点法（最新版）
class Gaofei_Navigation_ControlModel :public IdealControlModel {
private:
	MovementLaw guidance_station_movement_;
	MovementLaw target_movement_;
public:

	Gaofei_Navigation_ControlModel(double t_start) {
		this->t_start_ = t_start;


		distance_of_stop_nav = 0; //末端停止比例导引的弹目距离？

		distance_of_end = 0.3;  //停止计算的弹目距离， 弹道结束条件

	}
	void Load(Name2Value& n2vs, three_degree_trajectory* tj);
	bool LoadLaserFile(const char* filename) {
		FILE *f_laser;
		//double data_laser[79][2];
		//int i, j;
		//if ((f_laser = fopen("E:\\界面项目\\导引弹道（高飞）\\focus_position.dat", "r")) == NULL)
		if ((f_laser = fopen(filename, "r")) == NULL)
		{
			printf("focus_position.dat\n");
			exit(0);
		}

		double temp_t, temp_shecheng;
		while (!feof(f_laser)) {
			fscanf(f_laser, "%lf", &temp_t);
			fscanf(f_laser, "%lf", &temp_shecheng);
			this->arry_T.push_back(temp_t);
			this->arry_shecheng.push_back(temp_shecheng);

		}

	}

	//bool LoadGuidanceFile(const char* filename) {
	//	return guidance_station_movement_.Load(filename);
	//}
	//bool LoadTargetFile(const char* filename) {
	//	return target_movement_.Load(filename);

	//}

	std::vector<double> arry_T;
	std::vector<double> arry_shecheng;
	double R0;//光斑初始定焦距离
	double Hd;//理想高飞高度
	int flag_highfly;//判断是否采用高飞弹道，使用值为1，不使用值为0

	double distance_of_stop_nav; //末端停止比例导引的弹目距离？

	double distance_of_end;  //停止计算的弹目距离， 弹道结束条件

	double epsilon;

public:

	virtual void control_equations(double t, double dt, double y[], double* delta_y, double* delta_z, double* alpha, double*beta, double* gamma_v, three_degree_trajectory* tj);

	//弹道结束条件
	virtual bool is_trajectory_end(double t, double states[]);

public:
	double calc_r(double t, double states[]);


};

//class ThreePoint_Navigation_ControlModel : public IdealControlModel {
//public:
//	virtual void control_equations(double t, double dt, double y[], double* delta_y, double* delta_z, double* alpha, double*beta, double* gamma_v, three_degree_trajectory* tj) ;
//
//	//弹道是否可以结束（例如导弹已经命中目标），方案导引时一般不判断导弹是否结束
//	virtual bool is_trajectory_end(double t, double y[]);
//
//	void Load(Name2Value& n2vs);
//public:
//	//目标起始位置，t为target缩写
//	double y_t_0 = 0; 
//	double x_t_0 = 0;
//	double z_t_0 = 0;
//
//	//目标运动速度，对于航弹来说，假定目标只在水平面运动
//	double v_t_x = 0; 
//	double v_t_z = 0; 
//
//protected:
//	inline double x_t(double t) {
//		return x_t_0 + v_t_x*(t - t_start_);
//	}
//	inline double y_t(double t) {
//		return y_t_0 ;
//	}
//	inline double z_t(double t) {
//		return z_t_0 + v_t_z*(t - t_start_);
//	}
//
//
//};
//

//（速度）追踪法 , 可以按照比例导引的K=1来实现？
//class Pursuit_Navigation_ControlModel :public IdealControlModel {
//public:
//	Pursuit_Navigation_ControlModel(double t_start, double x_target, double y_target = 0, double z_target = 0, double v_target_x = 0, double v_target_z = 0) {
//		this->t_start_ = t_start;
//		this->y_t_0 = y_target;
//		this->x_t_0 = x_target;
//		this->z_t_0 = z_target;
//		this->v_t_x = v_target_x;
//		this->v_t_z = v_target_z;
//	}
//	Pursuit_Navigation_ControlModel(double t_start) {
//		this->t_start_ = t_start;
//	}
//
//
//	void Load(Name2Value& n2vs);
//	//目标起始位置，t为target缩写
//	double y_t_0 = 0;
//	double x_t_0 = 0;
//	double z_t_0 = 0;
//
//	//目标运动速度，对于航弹来说，假定目标只在水平面运动
//	double v_t_x = 0;
//	double v_t_z = 0;
//
//	double distance_of_stop_nav = 0; //末端停止追踪法的弹目距离？
//
//	double distance_of_end = 0.3;  //停止计算的弹目距离， 弹道结束条件
//
//public:
//
//	virtual void control_equations(double t, double dt, double y[], double* delta_y, double* delta_z, double* alpha, double*beta, double* gamma_v, three_degree_trajectory* tj);
//
//	//弹道结束条件
//	virtual bool is_trajectory_end(double t, double states[]);
//
//protected:
//	inline double x_t(double t) {
//		return x_t_0 + v_t_x*(t - t_start_);
//	}
//	inline double y_t(double t) {
//		return y_t_0;
//	}
//	inline double z_t(double t) {
//		return z_t_0 + v_t_z*(t - t_start_);
//	}
//
//	double calc_r(double t, double states[]);
//
//	//计算纵向(vertical)平面（弹道坐标系）内的dq/dt
//	double calc_dq_v_dt(double dx, double dy, double sigma, double sigma_t, double V, double V_t);
//	//计算水平面（horizontal)内的dq/dt
//	double calc_dq_h_dt(double dx, double dz, double sigma, double sigma_t, double V, double V_t, double& q_h, double& eta_h, double& r_h);
//};