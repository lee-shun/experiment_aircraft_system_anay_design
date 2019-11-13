//�������ģ�ͣ���Ӧ�����ɶ��ʵ㵯��
//���ݵ�ǰ״̬y[]������������ƹ��ɣ�����ϵͳ�������ӳ٣������㲢������Ӧ��״̬��gamma_v, alpha, beta, delta_z, delta_y, �Լ� theta,y�ȣ�
//Ϊ��ͨ���Լ���������� ��ƽ���ϵʽҲ�������������ģ���У�һ��
#pragma once
class three_degree_trajectory;

#include "stdafx.h"
#include "../interp.h"
#include "const.h"

class IdealControlModel {
protected:
double t_start_; //���ʱ��
public:
	~IdealControlModel() {};

	virtual void control_equations(double t, double dt, double y[], double* delta_y, double* delta_z, double* alpha, double*beta, double* gamma_v, three_degree_trajectory* tj) = 0;

	//�����Ƿ���Խ��������絼���Ѿ�����Ŀ�꣩����������ʱһ�㲻�жϵ����Ƿ����
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

	double platform_altitude_; //ƽ̨���θ߶ȣ���Ϊ������������ϵ����Ժ��Σ�


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

//��ƫ�ǿ���ģ�ͣ�ȥ�����޿�ģ�ͣ��ĳ��˶�ƫ�ǿ���ģ��)
class DeltaYZ_ControlModel :public IdealControlModel {
public:
	DeltaYZ_ControlModel(double t_start) {
		t_start_ = t_start;
	}

	//������ƫ��ʱ������
	std::vector<double> delta_z_t_;
	std::vector<double> delta_z_;

	//ƫ����ƫ��ʱ������
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



	//����ʱ������
	std::vector<double> alpha_t_array;
	std::vector<double> alpha_array;

	//�໬��ʱ������
	std::vector<double> beta_t_array;
	std::vector<double> beta_array;

	//�໬��ʱ������
	std::vector<double> gammav_t_array;
	std::vector<double> gammav_array;

public:
	void Load(Name2Value& n2vs);
public:
	virtual void control_equations(double t, double dt, double y[], double* delta_y, double* delta_z, double* alpha, double*beta, double* gamma_v, three_degree_trajectory* tj) ;

};

//�������/ƫ���������ģ��
class ThetaPsiV_ControlModel :public IdealControlModel {
public:
	ThetaPsiV_ControlModel(double t_start) {
		t_start_ = t_start;
	}

	//���ʱ������
	std::vector<double> theta_t_array;
	std::vector<double> theta_array;

	//ƫ��ʱ������
	std::vector<double> psi_v_t_array;
	std::vector<double> psi_v_array;

void Load(Name2Value& n2vs);
protected:
	double Theta(double t);
	double Psi_V(double t);
public:
	virtual void control_equations(double t, double dt, double y[], double* delta_y, double* delta_z, double* alpha, double*beta, double* gamma_v, three_degree_trajectory* tj) ;

};

//������/ƫ�����������ģ��
class VarTheta_Psi_ControlModel :public IdealControlModel {
public:
	VarTheta_Psi_ControlModel(double t_start) {
		t_start_ = t_start;
	}

	//������ʱ������
	std::vector<double> vartheta_t_array;
	std::vector<double> vartheta_array;

	//ƫ��ʱ������
	std::vector<double> psi_t_array;
	std::vector<double> psi_array;

void Load(Name2Value& n2vs);
protected:
	double VarTheta(double t);
	double Psi(double t);
public:
	virtual void control_equations(double t, double dt, double y[], double* delta_y, double* delta_z, double* alpha, double*beta, double* gamma_v, three_degree_trajectory* tj) ;

};



//���ؿ���ģ��
class Ny2Nz2_ControlModel :public IdealControlModel {
public:
	Ny2Nz2_ControlModel(double t_start) {
		t_start_ = t_start;
	}

	//��������ʱ������
	std::vector<double> ny2_t_array;
	std::vector<double> ny2_array;

	//ƫ��ʱ������
	std::vector<double> nz2_t_array;
	std::vector<double> nz2_array;

	void Load(Name2Value& n2vs);
protected:
	double Ny2(double t);
	double Nz2(double t);
public:
	virtual void control_equations(double t, double dt, double y[], double* delta_y, double* delta_z, double* alpha, double*beta, double* gamma_v, three_degree_trajectory* tj);

};


//����/���������������ģ�� ������������ʱҲ��û��ʹ�õģ���
class YZ_ControlModel :public IdealControlModel {
public:
	YZ_ControlModel(double t_start) {
		t_start_ = t_start;
	}

	//��������ʱ������
	std::vector<double> y_t_array;
	std::vector<double> y_array;

	//��������ʱ������
	std::vector<double> z_t_array;
	std::vector<double> z_array;

void Load(Name2Value& n2vs);
protected:
	double H(double t);
	double Z(double t);
public:
	virtual void control_equations(double t, double dt, double y[], double* delta_y, double* delta_z, double* alpha, double*beta, double* gamma_v, three_degree_trajectory* tj) ;

};

//XYZ���Ŀ���ģ��
class XYZ_ControlModel :public IdealControlModel {
public:
	XYZ_ControlModel(double t_start) {
		t_start_ = t_start;
	}


	//��������
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


//��������������������ģ��
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
	//��������ϵ��
	double K;


	MovementLaw target_movement_;



	double distance_of_stop_nav = 0; //ĩ��ֹͣ���������ĵ�Ŀ���룿

	double distance_of_end = 1.5;  //ֹͣ����ĵ�Ŀ���룬 ������������

public:

	virtual void control_equations(double t, double dt, double y[], double* delta_y, double* delta_z, double* alpha, double*beta, double* gamma_v, three_degree_trajectory* tj) ;

	//������������
	virtual bool is_trajectory_end(double t, double states[]);

protected:


	double calc_r(double t, double states[]) ;

	//��������(vertical)ƽ�棨��������ϵ���ڵ�dq/dt
	double calc_dq_v_dt(double dx, double dy, double sigma, double sigma_t, double V, double V_t, double& q);
	//����ˮƽ�棨horizontal)�ڵ�dq/dt
	double calc_dq_h_dt(double dx, double dz, double sigma, double sigma_t, double V, double V_t, double& q_h, double& eta_h, double& r_h) ;
};


class Threepoint_Navigation_ControlModel :public IdealControlModel {
private:
	MovementLaw guidance_station_movement_;
	MovementLaw target_movement_;
public:

	Threepoint_Navigation_ControlModel(double t_start) {
		this->t_start_ = t_start;



		//distance_of_stop_nav = 0; //ĩ��ֹͣ�����ĵ�Ŀ���룿

		distance_of_end = 0.6;  //ֹͣ����ĵ�Ŀ���룬 ������������
	}



	//double distance_of_stop_nav; //ĩ��ֹͣ���������ĵ�Ŀ���룿

	double distance_of_end;  //ֹͣ����ĵ�Ŀ���룬 ������������


	//double epsilon;

public:

	void Load(Name2Value& n2vs, three_degree_trajectory* tj);

	virtual void control_equations(double t, double dt, double y[], double* delta_y, double* delta_z, double* alpha, double*beta, double* gamma_v, three_degree_trajectory* tj);

	//������������
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

		//Ŀ����ʼλ�ã�tΪtarget��д
		//y_t_0 = 0;
		////x_t_0 = 12000;
		//x_t_0 = 0;
		//z_t_0 = 0;

		//�Ƶ�վ��ʼλ�ã�cΪcontrol��д
		//y_c_0 = 12000;
		//y_c_0 = 0;
		//x_c_0 = 0;
		//z_c_0 = 0;

		////Ŀ���˶��ٶȣ����ں�����˵���ٶ�Ŀ��ֻ��ˮƽ���˶�
		//v_t_x = 0;
		//v_t_z = 0;

		////�Ƶ�վ�˶��ٶȣ�
		//v_c_x = 0;
		//v_c_y = 0;
		//v_c_z = 0;

//		distance_of_stop_nav = 0; //ĩ��ֹͣ���������ĵ�Ŀ���룿

		distance_of_end = 0.6;  //ֹͣ����ĵ�Ŀ���룬 ������������
	}



	//Ŀ����ʼλ�ã�tΪtarget��д
	//double y_t_0;
	//double x_t_0;
	//double z_t_0;
	//�Ƶ�վ��ʼλ�ã�cΪcontrol��д
	//double y_c_0;
	//double x_c_0;
	//double z_c_0;

	//Ŀ���˶��ٶȣ����ں�����˵���ٶ�Ŀ��ֻ��ˮƽ���˶�
	//double v_t_x;
	//double v_t_z;
	//�Ƶ�վ�˶��ٶȣ�
	//double v_c_x;
	//double v_c_y;
	//double v_c_z;

	//ˮƽǰ�ý�
	double d_epsilon0_h = 0;// / RAD;
	//����ǰ�ý�
	double d_epsilon0_v = 0;// 10 / RAD;

//	double distance_of_stop_nav; //ĩ��ֹͣ���������ĵ�Ŀ���룿

	double distance_of_end;  //ֹͣ����ĵ�Ŀ���룬 ������������

	//double epsilon;

public:
	void Load(Name2Value& n2vs, three_degree_trajectory* tj);

	virtual void control_equations(double t, double dt, double y[], double* delta_y, double* delta_z, double* alpha, double*beta, double* gamma_v, three_degree_trajectory* tj);

	//������������
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





//�߷����㷨�����°棩
class Gaofei_Navigation_ControlModel :public IdealControlModel {
private:
	MovementLaw guidance_station_movement_;
	MovementLaw target_movement_;
public:

	Gaofei_Navigation_ControlModel(double t_start) {
		this->t_start_ = t_start;


		distance_of_stop_nav = 0; //ĩ��ֹͣ���������ĵ�Ŀ���룿

		distance_of_end = 0.3;  //ֹͣ����ĵ�Ŀ���룬 ������������

	}
	void Load(Name2Value& n2vs, three_degree_trajectory* tj);
	bool LoadLaserFile(const char* filename) {
		FILE *f_laser;
		//double data_laser[79][2];
		//int i, j;
		//if ((f_laser = fopen("E:\\������Ŀ\\�����������߷ɣ�\\focus_position.dat", "r")) == NULL)
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
	double R0;//��߳�ʼ��������
	double Hd;//����߷ɸ߶�
	int flag_highfly;//�ж��Ƿ���ø߷ɵ�����ʹ��ֵΪ1����ʹ��ֵΪ0

	double distance_of_stop_nav; //ĩ��ֹͣ���������ĵ�Ŀ���룿

	double distance_of_end;  //ֹͣ����ĵ�Ŀ���룬 ������������

	double epsilon;

public:

	virtual void control_equations(double t, double dt, double y[], double* delta_y, double* delta_z, double* alpha, double*beta, double* gamma_v, three_degree_trajectory* tj);

	//������������
	virtual bool is_trajectory_end(double t, double states[]);

public:
	double calc_r(double t, double states[]);


};

//class ThreePoint_Navigation_ControlModel : public IdealControlModel {
//public:
//	virtual void control_equations(double t, double dt, double y[], double* delta_y, double* delta_z, double* alpha, double*beta, double* gamma_v, three_degree_trajectory* tj) ;
//
//	//�����Ƿ���Խ��������絼���Ѿ�����Ŀ�꣩����������ʱһ�㲻�жϵ����Ƿ����
//	virtual bool is_trajectory_end(double t, double y[]);
//
//	void Load(Name2Value& n2vs);
//public:
//	//Ŀ����ʼλ�ã�tΪtarget��д
//	double y_t_0 = 0; 
//	double x_t_0 = 0;
//	double z_t_0 = 0;
//
//	//Ŀ���˶��ٶȣ����ں�����˵���ٶ�Ŀ��ֻ��ˮƽ���˶�
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

//���ٶȣ�׷�ٷ� , ���԰��ձ���������K=1��ʵ�֣�
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
//	//Ŀ����ʼλ�ã�tΪtarget��д
//	double y_t_0 = 0;
//	double x_t_0 = 0;
//	double z_t_0 = 0;
//
//	//Ŀ���˶��ٶȣ����ں�����˵���ٶ�Ŀ��ֻ��ˮƽ���˶�
//	double v_t_x = 0;
//	double v_t_z = 0;
//
//	double distance_of_stop_nav = 0; //ĩ��ֹͣ׷�ٷ��ĵ�Ŀ���룿
//
//	double distance_of_end = 0.3;  //ֹͣ����ĵ�Ŀ���룬 ������������
//
//public:
//
//	virtual void control_equations(double t, double dt, double y[], double* delta_y, double* delta_z, double* alpha, double*beta, double* gamma_v, three_degree_trajectory* tj);
//
//	//������������
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
//	//��������(vertical)ƽ�棨��������ϵ���ڵ�dq/dt
//	double calc_dq_v_dt(double dx, double dy, double sigma, double sigma_t, double V, double V_t);
//	//����ˮƽ�棨horizontal)�ڵ�dq/dt
//	double calc_dq_h_dt(double dx, double dz, double sigma, double sigma_t, double V, double V_t, double& q_h, double& eta_h, double& r_h);
//};