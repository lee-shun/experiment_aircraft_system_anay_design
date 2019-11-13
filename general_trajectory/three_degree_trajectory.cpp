#include "stdafx.h"
#include "three_degree_trajectory.h"


three_degree_trajectory::three_degree_trajectory()
{
	memset(_states, 0, Number_RungeKutta*sizeof(double));
}


three_degree_trajectory::~three_degree_trajectory()
{

}

void three_degree_trajectory::output_q(FILE* f_q, double t) {
	fprintf(f_q, "%15.10f ", t);
	fprintf(f_q, "%15.10f ", q_h*RAD);
	fprintf(f_q, "%15.10f ", dq_h_dt*RAD);
	fprintf(f_q, "%15.10f ", q_m_h*RAD);
	fprintf(f_q, "%15.10f ", q_v*RAD);
	fprintf(f_q, "%15.10f ", dq_v_dt*RAD);
	fprintf(f_q, "%15.10f ", q_m_v*RAD);
	fprintf(f_q, "\n");
}

void three_degree_trajectory::output_title(FILE* fout, FILE* f_donglixishu, FILE* f_donglixishu_new, FILE* f_q){

	fprintf(f_q, "%s", "TIME(s)		q_h		dq_h_dt      q_m_h     q_v     dq_v_dt     q_m_v\n");

	fprintf(fout, "%s","Time(s)	      V	      Theta      	Psi_V       X       Y       Z       Mass       Gamma       DeltaY       DeltaZ        Alpha       Beta       Gamma_V       nx2       ny2       nz2        mach       ");
	fprintf(fout, "%s", "X       Y       Z       X0       Y0       Z0       X_delta       Y_delta        Z_delta       P       q       wx       DeltaY4       DeltaZ4       JX       YZ       S       L       mywx_beta");
	//fprintf(fout, "%s", "  cy/cx  deltay_roll  deltaz_roll");
	fprintf(fout, "%s", "  cy/cx ");
	fprintf(fout, "\n");
	
	fprintf(f_donglixishu, "%s", "TIME(s)       V       a22       a24       a25       a34       a35       ");
	fprintf(f_donglixishu, "%s", "b22       b24       b25       b34       b35       ");
	fprintf(f_donglixishu, "%s", "c11       c33       KDX       TDX       Km_a       Km_b       ny2       nz2       ");
	fprintf(f_donglixishu, "%s", "Wm       Xim       Wm_b       Xim_b       ");
	fprintf(f_donglixishu, "%s", "q       theta");
	fprintf(f_donglixishu, "\n");


	//新版完整的动力学系数
	fprintf(f_donglixishu_new, "%s", "TIME(s)       V       "); //[0,1]

	fprintf(f_donglixishu_new, "%s", "a11       a12       a13       a14       a15       a16       ");//[2,7]
	fprintf(f_donglixishu_new, "%s", "a21       a22       a23       a24       a24_       a25       a25_       a26       ");//[8,15]
	fprintf(f_donglixishu_new, "%s", "a31       a32       a33       a34       a35       a36       ");//[16,21]

	fprintf(f_donglixishu_new, "%s", "b11       b12       b13       b14       b15       b16       b17       b18       ");//[22,29]
	fprintf(f_donglixishu_new, "%s", "b21       b22       b23       b24       b24_       b25       b25_       b26       b27       b28       ");//[30,39]
	fprintf(f_donglixishu_new, "%s", "b31       b32       b33       b34       b35       b36       b37       b38       ");//[40,47]

	fprintf(f_donglixishu_new, "%s", "q       ny2       nz2       ");//[48,50]
	fprintf(f_donglixishu_new, "\n");

}

//激光变焦规律输出文件
void three_degree_trajectory::output_title_focus_motion_law(FILE* fout) {
	fprintf(fout, "%s", "t   Xfocus");
	fprintf(fout, "\n");
}

void three_degree_trajectory::output_focus_motion_law(FILE* fout, double t) {
	fprintf(fout, "%15.10f ", t);
	fprintf(fout, "%15.10f ", _states[X_Index]);
	fprintf(fout, "\n");
}

void three_degree_trajectory::result(FILE* fout, FILE* f_donglixishu, FILE* f_donglixishu_new, double t){
	result_trajectory(fout, t);
	result_donglixishu(f_donglixishu,f_donglixishu_new, t);
}

void  three_degree_trajectory::result_donglixishu(FILE* f_donglixishu, FILE* f_donglixishu_new, double t) {
	double const& V = _states[three_degree_trajectory::V_Index];
	double const& m = _states[three_degree_trajectory::Mass_Index];
	double const& theta = _states[three_degree_trajectory::Theta_Index];

	double const& vartheta = theta + _alpha; //假设侧向运动不大，俯仰角为弹道倾角+攻角 ， 未来应该准确的求出俯仰角（对于方案弹道来说，也是可以准确求出俯仰角和偏航角的。)

	//if (t >= 58.413) {
	//	int xxx = 0;
	//}

	//环境参数
	double altitude = _states[three_degree_trajectory::Y_Index]; //当前海拨高度
	double sonic = g_atmosphere_model.GetSonic(altitude);
	double rho = g_atmosphere_model.GetRHO(altitude);
	double mach = V / sonic;
	//double S = GetS(this->aero_data);
	//double L = GetL(this->aero_data);
	double S = this->_aero_model->GetS();
	double L = this->_aero_model->GetL();
	double L_MX = this->_aero_model->GetL_MX();
	double q = 0.5 * rho * V * V;
	double P = this->_thrust_model->get_current_thrust(t);

	const double delta_angle= 0.1/RAD;


	//!!!固定角度来求动力学系数，应亓国栋要求临时修改，不太确定这样修改的有效性。
	/*double temp_alpha = 2.5;
	double temp_beta = 2.5;
	double temp_delta_x = 0.5;
	double temp_delta_y = 0.5;
	double temp_delta_z = 0.5;*/

	double temp_alpha = _alpha;
	double temp_beta = _beta;
	double temp_delta_x = _delta_x;
	double temp_delta_y = _delta_y;
	double temp_delta_z = _delta_z;



	//北理飞行力学176页
	const double delta_mach = 0.01;
	double CX0 = this->_aero_model->GetCX(t, mach, temp_alpha, temp_beta, temp_delta_x, temp_delta_y, temp_delta_z);
	double CX1 = this->_aero_model->GetCX(t, mach + delta_mach, temp_alpha, temp_beta, temp_delta_x, temp_delta_y, temp_delta_z);
	double X_Mach = (CX1 - CX0)  * q * S / delta_mach; //阻力对速度的偏导数， 等价于阻力对马赫数的偏导数吗？
	double P_V = 0; //推力对速度的偏导数为0
	double a11 = (P_V - X_Mach) / m;

	double a12 = 0;

	double a13 = -G * cos(theta);

	CX1 = this->_aero_model->GetCX(t, mach , temp_alpha + delta_angle, temp_beta, temp_delta_x, temp_delta_y, temp_delta_z);
	double X_Alpha = (CX1 - CX0) * q * S / delta_angle;
	double P_Alpha = P*temp_alpha; //认为推力跟攻角无关
	double a14 = -1 * (X_Alpha + P_Alpha) / m;

	CX1 =  this->_aero_model->GetCX(t, mach , temp_alpha , temp_beta, temp_delta_x, temp_delta_y, temp_delta_z + delta_angle);
	double X_DeltaZ = (CX1 - CX0) * q * S / delta_angle;
	double a15 = -X_DeltaZ / m;

	double a16 = 1 / m;


	double k_dimensionless = L /(2 * V) ; //无量纲化系数k，datcom和国家标准均为 l/2v

	double Jz = this->_rotation_inertia->GetJZ(t);

	double mz0 = this->_aero_model->GetMZ(t, mach, temp_alpha, temp_beta, temp_delta_z);
	double mz1 =  this->_aero_model->GetMZ(t, mach + delta_mach, temp_alpha, temp_beta, temp_delta_z);
	double MZ_V = (mz1 - mz0) *  q * S * L * k_dimensionless * RAD / delta_mach;
	double a21 = MZ_V / Jz;

	double mzwz = this->_aero_model->GetMZWZ(t, mach, temp_alpha, temp_beta, temp_delta_z);
	double MZWZ = mzwz * q * S * L_MX * k_dimensionless * RAD;
	double a22 = MZWZ / Jz;

	double a23 = 0;

	if (t > 50.633)
	{
		double test = 1;
	}
	//if (t > 50.839)
	if (t > 50.820)
	{
		double test = 1;
	}
	//double
	double MZ_alpha = this->_aero_model->GetMZ_Alpha(t, mach, temp_alpha, temp_beta)  * q*S*L ;
	double a24 = MZ_alpha / Jz;

	double a24_ = 0; // 暂时不考虑下洗， 取a24'为0

	double MZ_deltaz = this->_aero_model->GetMZ_DeltaZ(t, mach, temp_alpha, temp_beta, temp_delta_z) * q*S*L;
	double a25 = MZ_deltaz  / Jz;
	double a25_ = 0; //暂时不考虑DeltaZ的导数对俯仰力矩的影响

	double a26 = 1 / Jz;

	double CY0 = this->_aero_model->GetCY(t, mach, temp_alpha, temp_beta, temp_delta_x, temp_delta_y, temp_delta_z);
	double CY1 = this->_aero_model->GetCY(t, mach + delta_mach, temp_alpha, temp_beta, temp_delta_x, temp_delta_y, temp_delta_z);
	double Y_Mach = (CY1 - CY0) * q * S / delta_mach;
	double a31 = (P_V * temp_alpha + Y_Mach) / (m * V);

	double a32 = 0;

	double a33 = G * sin(theta) / V;

	double Y_alpha = this->_aero_model->GetCY_Alpha(t, mach, temp_alpha, temp_beta) * q *S;
	double a34 = (P+Y_alpha) / (m*V);

	double Y_deltaz = this->_aero_model->GetCY_DeltaZ(t, mach, temp_alpha, temp_beta, temp_delta_z)*q*S;
	double a35 = Y_deltaz / (m*V);

	double a36 = 1 / (m*V);

	// 北理飞行力学 P232页
	double Jx = this->_rotation_inertia->GetJX(t);
	double Jy = this->_rotation_inertia->GetJY(t);

	double mxwx = this->_aero_model->GetMXWX(t, mach, temp_alpha, temp_beta, temp_delta_x);
	double MXWX = mxwx *  q * S * L * k_dimensionless * RAD;
	double b11 = MXWX / Jx;

	double MXWY = 0; //暂时不考虑交叉干扰
	double b12 = MXWY / Jx;

	double b13 = 0;

	double MX_Beta = 0;//暂时认为侧滑角不会导致滚转力矩
	double b14 = 0;

	double MX_DeltaY = 0;
	double b15 = MX_DeltaY / Jx;

	double b16 = 0;
	
	double mx0 = this->_aero_model->GetMX(t, mach, temp_alpha, temp_beta, temp_delta_x);
	double mx1 =  this->_aero_model->GetMX(t, mach, temp_alpha, temp_beta, temp_delta_x + delta_angle);
	double MX_DeltaX = ( mx1 - mx0) *  q * S * L_MX / delta_angle;
	double b17 = MX_DeltaX / Jx;

	double b18 = 1 / Jx;


	double MYWX = 0; //暂时不考虑WX造成的偏航力矩
	double b21 = MYWX / Jy;

	double mywy = this->_aero_model->GetMYWY(t, mach, temp_alpha, temp_beta, temp_delta_y);
	double MYWY = mywy * q * S * L * k_dimensionless * RAD;
	double b22 = MYWY / Jy;

	double b23 = 0;

	if (t > 43.083)
	{
		double test = 1;
	}
	double MY_beta = this->_aero_model->GetMY_Beta(t, mach, temp_alpha, temp_beta)  * q*S*L ;
	double b24 = MY_beta / Jy;

	double MY_DBeta = 0; //暂时不考虑Beta的变化率造成的偏航力矩
	double b24_ = MY_DBeta / Jy;


	double MY_deltay = this->_aero_model->GetMY_DeltaY(t, mach, temp_alpha, temp_beta, temp_delta_y) * q*S*L;
	double b25 = MY_deltay  / Jy;

	double MY_DDeltaY = 0; //暂时不考虑偏航舵偏角的变化率造成的偏航力矩
	double b25_ = MY_DDeltaY / Jy;

	double b26 = 0;

	double b27 = 0;

	double b28 = 1 / Jy;

	double b31 = 0;
	
	double b32 = -cos(theta) / cos(vartheta);

	double b33 = 0;

	double Z_beta = this->_aero_model->GetCZ_Beta(t, mach, temp_alpha, temp_beta) * q *S;
	double b34 = (P-Z_beta) / (m*V);

	double Z_deltay = this->_aero_model->GetCZ_DeltaY(t, mach, temp_alpha, temp_beta, temp_delta_y)*q*S;
	double b35 = -Z_deltay / (m*V);

	double b36 = -G*cos(vartheta) / V;

	double b37 = 0;

	double b38 = -1 / (m*V);

	//double Jx = this->_rotation_inertia->GetJX(t);

	
	double c11 = 0;
	{
		double mxwx = this->_aero_model->GetMXWX(t, mach, temp_alpha, temp_beta, temp_delta_x);
		double MXWX = mxwx * q * S * L_MX * k_dimensionless * RAD;
		c11 = -MXWX/Jx;  //-b11
	}
	double MX_deltax = this->_aero_model->GetMX_DeltaX(t, mach, temp_alpha, temp_beta, temp_delta_x) * q*S*L_MX;
	double c33 = -MX_deltax / Jx; //-b17 

	double KDX = 0;
	double TDX = 0;
	if (c11 != 0) {
		KDX = -c33 / c11;
		TDX = 1 / c11;
	}

	//纵向传递系数
	double Km_a = V * (a25 *  a34 - a24 * a35) / (-a22*a34 - a24);
	//侧向传递系数
	double Km_b = 0;
	if( (-b22*b34 - b24) != 0 ) 
		Km_b = V * (b25 *  b34 - b24 * b35) / (-b22*b34 - b24);
	double Wm = 0; //纵向固有频率
	double Wm_2 = -a22 * a34 - a24;
	if(Wm_2 > 0)
		Wm = sqrt(Wm_2);
	double Xim = 0; //纵向阻尼
	if(Wm != 0)
		Xim = (a34 - a22) / (2 * Wm);
	double Wm_b = 0;
	double Wm_b_2 = -b22 * b34 - b24;
	if(Wm_b_2 > 0)
		Wm_b = sqrt(Wm_b_2);
	double Xim_b = 0;
	if(Wm_b != 0)
		Xim_b = (b34 - b22) / (2 * Wm_b);

	fprintf(f_donglixishu, "%lf       %lf       ", t, _states[V_Index]);
	fprintf(f_donglixishu, "%lf       %lf       %lf       %lf       %lf       ", a22, a24, a25, a34, a35);
	fprintf(f_donglixishu, "%lf       %lf       %lf       %lf       %lf       ", b22, b24, b25, b34, b35);
	fprintf(f_donglixishu, "%lf       %lf       ",  c11, c33);
	fprintf(f_donglixishu, "%lf       %lf        ", KDX, TDX);
	fprintf(f_donglixishu, "%lf       %lf       ", Km_a, Km_b);

	//Mat_mul_vec(Cvt, Force, Force_t);
	//Mat_mul_vec(Cbt, P, P_t);

	fprintf(f_donglixishu, "%lf       %lf       ", _ny2, _nz2);
	//fprintf(fout, "%lf       %lf       ", mz_alpha, mz_deltaZ);
	fprintf(f_donglixishu, "%lf       %lf       ", Wm, Xim);
	fprintf(f_donglixishu, "%lf       %lf       ", Wm_b, Xim_b);
	fprintf(f_donglixishu, "%lf       %lf       ", q,_states[Theta_Index]*RAD);
	fprintf(f_donglixishu, "\n");


	//新版完整动力系数输出 
	fprintf(f_donglixishu_new, "%lf       %lf       ", t, _states[V_Index]);
	fprintf(f_donglixishu_new,  "%lf       %lf       %lf       %lf       %lf       %lf       ", a11, a12, a13, a14, a15, a16);//[2,7]
	fprintf(f_donglixishu_new,  "%lf       %lf       %lf       %lf       %lf       %lf       %lf       %lf       ", a21, a22, a23, a24, a24_, a25, a25_, a26);//[8,15]
	fprintf(f_donglixishu_new,  "%lf       %lf       %lf       %lf       %lf       %lf       ", a31, a32, a33, a34, a35, a36);//[16,21]

	fprintf(f_donglixishu_new,  "%lf       %lf       %lf       %lf       %lf       %lf       %lf       %lf       ", b11, b12, b13, b14, b15, b16, b17, b18);//[22,29]
	fprintf(f_donglixishu_new,  "%lf       %lf       %lf       %lf       %lf       %lf       %lf       %lf       %lf       %lf       ", b21, b22, b23, b24, b24_, b25, b25_, b26, b27, b28);//[30,39]
	fprintf(f_donglixishu_new,  "%lf       %lf       %lf       %lf       %lf       %lf       %lf       %lf       ", b33, b32, b33, b34, b35, b36, b37, b38);//[40,47]

	fprintf(f_donglixishu_new,  "%lf       %lf       %lf", q, _ny2, _nz2);//[48]
	fprintf(f_donglixishu_new, "\n");


}


void three_degree_trajectory::result_trajectory(FILE* fout, double t) {
	double Jx = this->_rotation_inertia->GetJX(t);
	double Jy = this->_rotation_inertia->GetJY(t);
	double Jz = this->_rotation_inertia->GetJZ(t);
	double S = this->_aero_model->GetS();
	double L = this->_aero_model->GetL();
	fprintf(fout, "%lf       ", t);//1
	//用于输出的y
	double output_y[Number_RungeKutta];
	std::copy(_states, _states + Number_RungeKutta, output_y);
	//将绝对高度减去平台海拔，转换为相对高度
	output_y[Y_Index] -= platform_altitude_ ;
	//将用于输出的y转换为角度
	output_y[Theta_Index] = output_y[Theta_Index] * RAD;
	output_y[Psi_V_Index] = output_y[Psi_V_Index] * RAD;
	//output_y[Gamma_Index] = output_y[Gamma_Index] * RAD;
	for (int i = 0; i < Number_RungeKutta; i++)
		fprintf(fout, "%lf       ", output_y[i]);//2-8

	double Dummy_Gamma = 0; //为了兼容历史问题，输出的一个滚转角，实际上在方案弹道中无法计算滚转角
	fprintf(fout, "%lf       ", Dummy_Gamma);//9

	fprintf(fout, "%lf       ", _delta_y*RAD);//10 , 还是正常输出准弹体系的舵偏角，实际弹体系（滚转）下的舵偏角，到最后输出
	fprintf(fout, "%lf       ", _delta_z*RAD);//11
	fprintf(fout, "%lf       ", _alpha*RAD);//12
	fprintf(fout, "%lf       ", _beta*RAD);//13
	fprintf(fout, "%lf       ", _gamma_v*RAD);//14
	fprintf(fout, "%lf       ", _nx2);//15
	fprintf(fout, "%lf       ", _ny2);//16
	fprintf(fout, "%lf       ", _nz2);//17
	fprintf(fout, "%lf       ", _mach);//18


	//计算升力/阻力/侧向力 等进行输出：
	double V = _states[V_Index];
	double altitude = _states[Y_Index];
	double sonic = g_atmosphere_model.GetSonic(altitude);

	double 		rho = g_atmosphere_model.GetRHO(altitude);
	double X, Y, Z;
	this->_aero_model->calc_aerodynamic_force(t, V, sonic, rho, _alpha, _beta, _delta_x, _delta_y, _delta_z, X, Y, Z);
	double X0, Y0, Z0;
	this->_aero_model->calc_aerodynamic_force(t, V, sonic, rho, _alpha, _beta, 0, 0, 0, X0, Y0, Z0);

	fprintf(fout, "%lf       ", X);//19
	fprintf(fout, "%lf       ", Y);//20
	fprintf(fout, "%lf       ", Z);//21

	fprintf(fout, "%lf       ", X0);//22
	fprintf(fout, "%lf       ", Y0);//23
	fprintf(fout, "%lf       ", Z0);//24


	fprintf(fout, "%lf       ", X-X0);//25
	fprintf(fout, "%lf       ", Y-Y0);//26
	fprintf(fout, "%lf       ", Z-Z0);//27

	double P = this->_thrust_model->get_current_thrust(t);
	fprintf(fout, "%lf       ", P); //28

	double q = 0.5 * rho * V * V;
	fprintf(fout, "%lf       ", q); //29
	fprintf(fout, "%lf       ", wx_/(2*3.14159));//30 , 由弧度/秒转化为转/秒
	if (flag_type == 1)
	{
		fprintf(fout, "%lf       ", 0.);//31
		fprintf(fout, "%lf       ", 0.);//32
	}
	else
	{
		fprintf(fout, "%lf       ", _delta_y*RAD);//31
		fprintf(fout, "%lf       ", _delta_z*RAD);//32
	}

	fprintf(fout, "%lf       ", Jx); //33
	fprintf(fout, "%lf       ", Jz); //34 
	fprintf(fout, "%lf       ", S);  //35
	fprintf(fout, "%lf       ", L);  //36

	double mywx_beta;
	mywx_beta = this->_aero_model->GetMYWX_Beta_Rad(t, _mach, _alpha, _beta);
	fprintf(fout, "%lf       ", mywx_beta);//37

	fprintf(fout, "%lf       ", Y/X);//38 , 升阻比
	//fprintf(fout, "%lf       ", deltay_roll*RAD);//39
	//fprintf(fout, "%lf       ", deltaz_roll*RAD);//40

	fprintf(fout, "\n");

}

void three_degree_trajectory::output_zhixin(FILE* f_zhixin, double t)
{
	fprintf(f_zhixin, "%15.10f ", t);
	fprintf(f_zhixin, "%15.10f ", _states[Y_Index]);
	fprintf(f_zhixin, "%15.10f ", _states[Z_Index]);
	fprintf(f_zhixin, "\n");
}




void three_degree_trajectory::output_result2(FILE* f_result2, double t) {
	double Jx = this->_rotation_inertia->GetJX(t);
	double Jy = this->_rotation_inertia->GetJY(t);
	double Jz = this->_rotation_inertia->GetJZ(t);
	double S = this->_aero_model->GetS();
	double L = this->_aero_model->GetL();
	fprintf(f_result2, "%15.10f ", t);//1
	//用于输出的y
	double output_y[Number_RungeKutta];
	std::copy(_states, _states + Number_RungeKutta, output_y);
	//将用于输出的y转换为角度
	output_y[Theta_Index] = output_y[Theta_Index] * RAD;
	output_y[Psi_V_Index] = output_y[Psi_V_Index] * RAD;
//	output_y[Gamma_Index] = output_y[Gamma_Index] * RAD;
	for (int i = 0; i < Number_RungeKutta; i++)
		fprintf(f_result2, "%15.10f ", output_y[i]);//2-8

	double Dummy_Gamma = 0; //为了兼容历史问题，输出的一个滚转角，实际上在方案弹道中无法计算滚转角
	fprintf(f_result2, "%lf       ", Dummy_Gamma);//9

	fprintf(f_result2, "%15.10f ", _delta_y*RAD);//10
	fprintf(f_result2, "%15.10f ", _delta_z*RAD);//11
	fprintf(f_result2, "%15.10f ", _alpha*RAD);//12
	fprintf(f_result2, "%15.10f ", _beta*RAD);//13
	fprintf(f_result2, "%15.10f ", _gamma_v*RAD);//14
	fprintf(f_result2, "%15.10f ", _nx2);//15
	fprintf(f_result2, "%15.10f ", _ny2);//16
	fprintf(f_result2, "%15.10f ", _nz2);//17
	fprintf(f_result2, "%15.10f ", _mach);//18

	//计算升力/阻力/侧向力 等进行输出：
	double V = _states[V_Index];
	double altitude = _states[Y_Index];
	double sonic = g_atmosphere_model.GetSonic(altitude);

	double 		rho = g_atmosphere_model.GetRHO(altitude);
	double X, Y, Z;
	this->_aero_model->calc_aerodynamic_force(t, V, sonic, rho, _alpha, _beta, _delta_x, _delta_y, _delta_z, X, Y, Z);
	double X0, Y0, Z0;
	this->_aero_model->calc_aerodynamic_force(t, V, sonic, rho, _alpha, _beta, 0, 0, 0, X0, Y0, Z0);

	fprintf(f_result2, "%15.10f ", X);//19
	fprintf(f_result2, "%15.10f ", Y);//20
	fprintf(f_result2, "%15.10f ", Z);//21

	fprintf(f_result2, "%15.10f ", X0);//22
	fprintf(f_result2, "%15.10f ", Y0);//23
	fprintf(f_result2, "%15.10f ", Z0);//24


	fprintf(f_result2, "%15.10f ", X - X0);//25
	fprintf(f_result2, "%15.10f ", Y - Y0);//26
	fprintf(f_result2, "%15.10f ", Z - Z0);//27

	double P = this->_thrust_model->get_current_thrust(t);
	fprintf(f_result2, "%15.10f ", P); //28

	double q = 0.5 * rho * V * V;
	fprintf(f_result2, "%15.10f ", q); //29
	fprintf(f_result2, "%15.10f ", wx_/(2*3.14159));//30, 由弧度/秒转化为转/秒
	if (flag_type == 1)
	{
		fprintf(f_result2, "%15.10f ", 0.);//31
		fprintf(f_result2, "%15.10f ", 0.);//32
	}
	else
	{
		fprintf(f_result2, "%15.10f ", _delta_y*RAD);//31
		fprintf(f_result2, "%15.10f ", _delta_z*RAD);//32
	}

	fprintf(f_result2, "%15.10f ", Jx);
	fprintf(f_result2, "%15.10f ", Jz);
	fprintf(f_result2, "%15.10f ", S);
	fprintf(f_result2, "%15.10f ", L);

	double mywx_beta;
	mywx_beta = this->_aero_model->GetMYWX_Beta_Rad(t, _mach, _alpha, _beta);
	fprintf(f_result2, "%15.10f ", mywx_beta);

	fprintf(f_result2, "\n");

}
