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


//���ģ������ɶȣ�����
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
		//Delta_P //Ŀǰ����������һ�㲻�ɵ��ڣ����ȡ������һ��
		//����״̬�����ڻ��ַ��̣����������������
		Number_RungeKutta, //��������������ֵı������� 
		//				   //DeltaX_Index, //�ʵ㵯�������ǹ�ת��ƫ�ǣ�Ҳ�޷�����
		//DeltaY_Index = Number_RungeKutta, //ƫ����ƫ����ƽ���ϵʽֱ�Ӽ��㣬��΢�ַ��̣���˲���Ҫ���������������
		//DeltaZ_Index, //������ƫ����ƽ���ϵʽֱ�Ӽ��㣬��΢�ַ��̣���˲���Ҫ���������������
		//Alpha_Index,
		//Beta_Index,
		//GammaV_Index,
		//NY2_Index, //�������
		//NZ2_Index, //�������
		//Number_States //״̬�����ĸ���
	};
	//�����������δ֪��������, ����Ƕ�ֵ�ĵ�λΪ���ȡ���֯��������ʽ��Ϊ�������������
	double _states[Number_RungeKutta];

	double& _V = _states[V_Index];
	double& _Theta = _states[Theta_Index];
	double& _Psi_V = _states[Psi_V_Index];
	double& _X = _states[X_Index];
	double& _Y = _states[Y_Index];
	double& _Z = _states[Z_Index];
	double& _Mass = _states[Mass_Index];
	//double& _Gamma = _states[Gamma_Index];



	//����������������ֵ�״̬����
	double _delta_y = 0;
	double _delta_z = 0;
	const double _delta_x = 0;  //��������ϲ������ʵ㵯���ļ��㣬���ǿ��ǽ�������mx_beta�Լ�mx0
	double _alpha = 0;
	double _beta = 0;
	double _gamma_v = 0;

	//����
	double _nx2 = 0;
	double _ny2 = 0;
	double _nz2 = 0;

	//�����
	double _mach = 0;

	//ƽ̨���Σ���������ϵ�ĸ����
	double platform_altitude_ = 0; //ƽ̨���Σ� ������Ŀ�ꡢ�Ƶ�վ��Y���꣬���������������

	double wx_; //�������ת�٣� ����ת���غ͹�ת�������ش��ƽ��ʱ��ת��
	int flag_type;
	int flag_channel;
	//double deltay_roll, deltaz_roll;

	//q_h��dq_h_dt��q_m_h��
	//��Ŀ���߽ǡ���Ŀʱ����ٶȡ���Ŀ���߽��뵯��н�(hΪˮƽƽ���ϣ�vΪ����ƽ�棩
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
	//TargetModel*  _target_model = NULL;  //Ŀ���˶�ģ��

	bool is_end(double t){//��������Ľ�������
		return _states[Y_Index] <= platform_altitude_ || this->_ideal_control_model->is_trajectory_end(t, _states);
	}

	bool near_end() {//��Ҫ����
		return _states[Y_Index] <= 20;
	}

	//����tʱ�̣� ��΢�ַ��̵�״̬��
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
		//������� 
		//�ȸ��ݵ�ǰ״̬��һ��dV/dt��dtheta/dt, dpsi_v/dt
		double dy_dt[Number_RungeKutta];
		this->trajectory_equations_internal(t, dt, _states, dy_dt);
		double dv_dt = dy_dt[V_Index];
		double dtheta_dt = dy_dt[Theta_Index];
		double dpsiv_dt = dy_dt[Psi_V_Index];

		//���ؼ��㹫ʽ������������ѧ��p60
		_nx2 = (1 / G)*dv_dt + sin(theta);
		_ny2 = (V / G)*dtheta_dt + cos(theta);
		_nz2 = -(V / G) * cos(theta) *dpsiv_dt;


		//�����ɶȵ������̵ķ�΢�ַ��̣� Ҳ�����������ģ�Ͷ�Ӧ�ķ��̣�����˲ʱƽ���ϵʽ��
		_ideal_control_model->control_equations(t,dt,_states, &_delta_y, &_delta_z, &_alpha, &_beta, &_gamma_v, this);

	}

private:

	//�����ɶȣ��ʵ㣩����������
	//����������ֵ��Ҷ˺����� ����˼���Ǹ��ݣ� t(k),y[k]   ->    y[k+1] 
	//Ϊ�˱��ڸ����ü�� �� �޸ĳ�static function
	inline static void three_degree_trajectory_equations(double dy_dt[], double t, double dt, double const y[], three_degree_trajectory* tj){
		//��Ӧ�������������������
		const double& V = y[V_Index]; //�ٶ�
		const double& theta = y[Theta_Index]; //�������
		const double& psi_v = y[Psi_V_Index]; //����ƫ��
		const double& x = y[X_Index];
		const double& y_ = y[Y_Index]; //Ϊ�˱������Ƴ�ͻ��������y_
		const double& z = y[Z_Index];
		const double& m = y[Mass_Index];

		//����ģ��, 
		double P = tj->_thrust_model->get_current_thrust(t); 

		double delta_y;
		double delta_z;
		double gamma_v;
		double alpha;
		double beta;

		double delta_x = 0;

		//�����������ģ�ͼ��㵱ǰ״̬������Ӧ�ģ��������ֵ
		tj->_ideal_control_model->control_equations(t, dt, (double*)y, &delta_y, &delta_z, &alpha, &beta, &gamma_v, tj);
		

		//��������
		double sonic = g_atmosphere_model.GetSonic(y_);
		double rho = g_atmosphere_model.GetRHO(y_);

		//����������
		//������ģ��
		double X, Y, Z;
		tj->_aero_model->calc_aerodynamic_force(t, V, sonic, rho, alpha, beta, 0, delta_y, delta_z, X, Y, Z);
		//aerodynamic_force_model_rad(tj->aero_data, V, sonic, rho, alpha, beta, 0, delta_y, delta_z, X, Y, Z);

		//�����˶�ģ��
		centroid_motion_equations(P, alpha, beta, gamma_v, X, Y, Z, V, theta,
			psi_v, x, y_, z, m, dy_dt[V_Index], dy_dt[Theta_Index], dy_dt[Psi_V_Index], dy_dt[X_Index],
			dy_dt[Y_Index], dy_dt[Z_Index]);

		//����ģ��
		//dy_dt[Mass_Index] = zero_mass_equations(t);
		dy_dt[Mass_Index] = tj->_mass_model->mass_flow_rate_equations(t);

		double mach_tj = V / sonic;
		if (tj->flag_type == 2 )
		{
			double mx, mxwx;
			mx = tj->_aero_model->GetMX(t, mach_tj, alpha, beta, delta_x);
			mxwx = tj->_aero_model->GetMXWX(t, mach_tj, alpha, beta, delta_x);
			double L_mx = tj->_aero_model->GetL_MX();
			//�������ۼ��޽��ٶȣ� ���ݹ�ʽ    mx + wxt*RAD*mxwx = 0 , 
			//����wxt*RAD����Ϊdatcom�Ĺ�ת����ϵ��mxwx�������ٻ�ʱ��ʹ�õ��ǡ���/�롱
			double wxt = -mx / (mxwx * RAD);//�����ٻ����ٶȣ�
			tj->wx_ = wxt / ( L_mx / (2*V) );//��ת���ٶȣ�����/��
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

		//������ƹ�ת
		//dy_dt[Gamma_Index] = tj->wx_;
	}



	//����������ֵ��Ҷ˺����� ����˼���Ǹ��ݣ� t(k),y[k], alpha(k), beta(k)...   ->    y[k+1] 
	virtual void trajectory_equations_internal(double t, double dt, double const y[], double dy_dt[]){
		three_degree_trajectory_equations(dy_dt, t, dt, y, this);
	}

//�����ش���
public:
	void output_title(FILE* fout, FILE* f_donglixishu, FILE* f_donglixishu_new, FILE* f_q);
	void result(FILE* fout, FILE* f_donglixishu, FILE* f_donglixishu_new, double t);
	void output_zhixin(FILE* f_zhixin, double t);
	void output_result2(FILE* f_result, double t);
	void output_title_focus_motion_law(FILE* fout);
	void output_focus_motion_law(FILE* fout, double t);
	//���Ŀ�귽λ�������Ϣ
	void output_q(FILE* f_q, double t);
protected: 
	void result_trajectory(FILE* fout, double t);
	void result_donglixishu(FILE* fout, FILE* f_donglixishu_new, double t);
};

