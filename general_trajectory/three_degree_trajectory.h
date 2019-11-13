#pragma once
#include "trajectory.h"
#include "const.h"
#include "ControlEquations.h"
#include "thrust_models.h"
#include "aerodynamic_model.h"
#include "CentroidMotionEquations.h"
#include "mass_model.h"
#include "ideal_control_models.h"
#include "target_model.h"


//质心（三自由度）弹道
class three_degree_trajectory :
	public trajectory
{
public:
	three_degree_trajectory();
	virtual ~three_degree_trajectory();

public:
	enum three_trajectory_state
	{
		V_Index,
		Theta_Index,
		Psi_V_Index,
		X_Index,
		Y_Index,
		Z_Index,
		Mass_Index,
		//Gamma_Index,
		//Delta_P //目前弹道中推力一般不可调节，因此取消了这一项
		//以上状态量属于积分方程，纳入龙格库塔计算
		Number_RungeKutta, //采用龙格库塔积分的变量个数 
		//				   //DeltaX_Index, //质点弹道不考虑滚转舵偏角，也无法计算
		//DeltaY_Index = Number_RungeKutta, //偏航舵偏角由平衡关系式直接计算，非微分方程，因此不需要纳入龙格库塔计算
		//DeltaZ_Index, //俯仰舵偏角由平衡关系式直接计算，非微分方程，因此不需要纳入龙格库塔计算
		//Alpha_Index,
		//Beta_Index,
		//GammaV_Index,
		//NY2_Index, //法向过载
		//NZ2_Index, //法向过载
		//Number_States //状态变量的个数
	};
	//弹道方程组的未知量的数组, 里面角度值的单位为弧度。组织成数组形式是为了龙格库塔积分
	double _states[Number_RungeKutta];

	double& _V = _states[V_Index];
	double& _Theta = _states[Theta_Index];
	double& _Psi_V = _states[Psi_V_Index];
	double& _X = _states[X_Index];
	double& _Y = _states[Y_Index];
	double& _Z = _states[Z_Index];
	double& _Mass = _states[Mass_Index];
	//double& _Gamma = _states[Gamma_Index];



	//不纳入龙格库塔积分的状态变量
	double _delta_y = 0;
	double _delta_z = 0;
	const double _delta_x = 0;  //这个基本上不纳入质点弹道的计算，除非考虑交叉力矩mx_beta以及mx0
	double _alpha = 0;
	double _beta = 0;
	double _gamma_v = 0;

	//过载
	double _nx2 = 0;
	double _ny2 = 0;
	double _nz2 = 0;

	//马赫数
	double _mach = 0;

	//平台海拔（发射坐标系的概念？）
	double platform_altitude_ = 0; //平台海拔， 导弹、目标、制导站的Y坐标，都将基于这个海拔

	double wx_; //理论最大转速， 当滚转力矩和滚转阻尼力矩达成平衡时的转速
	int flag_type;
	int flag_channel;
	//double deltay_roll, deltaz_roll;

	//q_h，dq_h_dt，q_m_h，
	//弹目视线角、弹目时间角速度、弹目视线角与弹轴夹角(h为水平平面上，v为纵向平面）
	double q_h=0;
	double dq_h_dt=0; 
	double q_v=0; 
	double dq_v_dt=0; 
	double q_m_h=0; 
	double q_m_v=0;




	virtual int get_number_of_runge_kutta(){ return Number_RungeKutta; }

public: 
	MassFlowRateModel* _mass_model = NULL;
	IdealControlModel* _ideal_control_model = NULL;
	ThrustModel* _thrust_model = NULL;
	//TargetModel*  _target_model = NULL;  //目标运动模型

	bool is_end(double t){//弹道计算的结束条件
		return _states[Y_Index] <= platform_altitude_ || this->_ideal_control_model->is_trajectory_end(t, _states);
	}

	bool near_end() {//快要结束
		return _states[Y_Index] <= 20;
	}

	//计算t时刻， 非微分方程的状态量
	virtual void no_differential_calculus_equations(double t, double dt) {
		double theta = _states[Theta_Index];
		double V = _states[V_Index];
		double altitude = _states[Y_Index];
		double sonic = g_atmosphere_model.GetSonic(altitude);
		_mach = V / sonic;

		if (t > 9.569)
		{
			double dummy = 0;
		}
		//计算过载 
		//先根据当前状态求一次dV/dt，dtheta/dt, dpsi_v/dt
		double dy_dt[Number_RungeKutta];
		this->trajectory_equations_internal(t, dt, _states, dy_dt);
		double dv_dt = dy_dt[V_Index];
		double dtheta_dt = dy_dt[Theta_Index];
		double dpsiv_dt = dy_dt[Psi_V_Index];

		//过载计算公式，北理工飞行力学，p60
		_nx2 = (1 / G)*dv_dt + sin(theta);
		_ny2 = (V / G)*dtheta_dt + cos(theta);
		_nz2 = -(V / G) * cos(theta) *dpsiv_dt;


		//三自由度弹道方程的非微分方程， 也就是理想控制模型对应的方程（包括瞬时平衡关系式）
		_ideal_control_model->control_equations(t,dt,_states, &_delta_y, &_delta_z, &_alpha, &_beta, &_gamma_v, this);

	}

private:

	//三自由度（质点）弹道方程组
	//龙格库塔积分的右端函数， 中心思想是根据： t(k),y[k]   ->    y[k+1] 
	//为了便于副作用检查 ， 修改成static function
	inline static void three_degree_trajectory_equations(double dy_dt[], double t, double dt, double const y[], three_degree_trajectory* tj){
		//对应上述方程组变量的引用
		const double& V = y[V_Index]; //速度
		const double& theta = y[Theta_Index]; //弹道倾角
		const double& psi_v = y[Psi_V_Index]; //弹道偏角
		const double& x = y[X_Index];
		const double& y_ = y[Y_Index]; //为了避免名称冲突，命名成y_
		const double& z = y[Z_Index];
		const double& m = y[Mass_Index];

		//推力模型, 
		double P = tj->_thrust_model->get_current_thrust(t); 

		double delta_y;
		double delta_z;
		double gamma_v;
		double alpha;
		double beta;

		double delta_x = 0;

		//根据理想控制模型计算当前状态量（对应的）理想控制值
		tj->_ideal_control_model->control_equations(t, dt, (double*)y, &delta_y, &delta_z, &alpha, &beta, &gamma_v, tj);
		

		//环境参数
		double sonic = g_atmosphere_model.GetSonic(y_);
		double rho = g_atmosphere_model.GetRHO(y_);

		//气动力计算
		//气动力模型
		double X, Y, Z;
		tj->_aero_model->calc_aerodynamic_force(t, V, sonic, rho, alpha, beta, 0, delta_y, delta_z, X, Y, Z);
		//aerodynamic_force_model_rad(tj->aero_data, V, sonic, rho, alpha, beta, 0, delta_y, delta_z, X, Y, Z);

		//质心运动模型
		centroid_motion_equations(P, alpha, beta, gamma_v, X, Y, Z, V, theta,
			psi_v, x, y_, z, m, dy_dt[V_Index], dy_dt[Theta_Index], dy_dt[Psi_V_Index], dy_dt[X_Index],
			dy_dt[Y_Index], dy_dt[Z_Index]);

		//质量模型
		//dy_dt[Mass_Index] = zero_mass_equations(t);
		dy_dt[Mass_Index] = tj->_mass_model->mass_flow_rate_equations(t);

		double mach_tj = V / sonic;
		if (tj->flag_type == 2 )
		{
			double mx, mxwx;
			mx = tj->_aero_model->GetMX(t, mach_tj, alpha, beta, delta_x);
			mxwx = tj->_aero_model->GetMXWX(t, mach_tj, alpha, beta, delta_x);
			double L_mx = tj->_aero_model->GetL_MX();
			//计算理论极限角速度， 根据公式    mx + wxt*RAD*mxwx = 0 , 
			//其中wxt*RAD是因为datcom的滚转阻尼系数mxwx在无量纲化时，使用的是“度/秒”
			double wxt = -mx / (mxwx * RAD);//无量纲化角速度，
			tj->wx_ = wxt / ( L_mx / (2*V) );//滚转角速度，弧度/秒
			//tj->deltay_roll = delta_y;
			//tj->deltaz_roll = delta_z;

			//if (tj->flag_channel == 1)
			//{
			//	//tj->deltaz_roll = delta_y*sin(y[Gamma_Index]) + delta_z*cos(y[Gamma_Index]);
			//	tj->deltay_roll = 0;
			//}
			//if (tj->flag_channel == 2)
			//{
			//	//tj->deltay_roll = delta_y*cos(y[Gamma_Index]) - delta_z*sin(y[Gamma_Index]);
			//	//tj->deltaz_roll = delta_y*sin(y[Gamma_Index]) + delta_z*cos(y[Gamma_Index]);
			//}
		}
		else if (tj->flag_type == 1)
		{
			tj->wx_ = 0;
			//tj->deltay_roll = delta_y;
			//tj->deltaz_roll = delta_z;
		}

		//计算近似滚转
		//dy_dt[Gamma_Index] = tj->wx_;
	}



	//龙格库塔积分的右端函数， 中心思想是根据： t(k),y[k], alpha(k), beta(k)...   ->    y[k+1] 
	virtual void trajectory_equations_internal(double t, double dt, double const y[], double dy_dt[]){
		three_degree_trajectory_equations(dy_dt, t, dt, y, this);
	}

//输出相关代码
public:
	void output_title(FILE* fout, FILE* f_donglixishu, FILE* f_donglixishu_new, FILE* f_q);
	void result(FILE* fout, FILE* f_donglixishu, FILE* f_donglixishu_new, double t);
	void output_zhixin(FILE* f_zhixin, double t);
	void output_result2(FILE* f_result, double t);
	void output_title_focus_motion_law(FILE* fout);
	void output_focus_motion_law(FILE* fout, double t);
	//输出目标方位角相关信息
	void output_q(FILE* f_q, double t);
protected: 
	void result_trajectory(FILE* fout, double t);
	void result_donglixishu(FILE* fout, FILE* f_donglixishu_new, double t);
};

