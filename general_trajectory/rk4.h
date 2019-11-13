#pragma once

//4�������������
//���ڶԶ�Ԫ΢�ַ�������л��֣�dy/dt = f(t,y)
//΢�ַ����飨δ֪�����ĸ���Ϊy_number, f(t,y)Ϊ�Ҷ˺���
//�ο����ϣ����������ѧ��#2-9 �����˶����������ֵ�ⷨ, 76ҳ
//http://rosettacode.org/wiki/Runge-Kutta_method
//

//�Ҷ˺��� , ����ͨ�Ҷ˺���������һ��������dt������ΪĳЩ���ƹ��ɵļ��������Ҫʱ�䲽��
typedef void (*RightFunction)(double t, double dt, double const y[], double dy_dt[], void* extern_data);

class rk4{
	int y_number = 0;//΢�ַ�����ĸ���
	std::vector<double> k1, k2, k3, k4; 
	std::vector<double> dy_dt;// dy/dt
	std::vector<double> y_temp; //�洢����k����ʱyֵ

public:
	rk4(int number){
		resize(number);
	}
	void resize(int number) {
		y_number = number;
		k1.resize(y_number);
		k2.resize(y_number);
		k3.resize(y_number);
		k4.resize(y_number);
		dy_dt.resize(y_number);
		y_temp.resize(y_number);
	}
	void run(RightFunction f, double dt, double t, double y[], void* extern_data){

		//k1 = dt * f(t, y)
		for (int i = 0; i < y_number; i++)
			y_temp[i] = y[i]; //����y_temp = y ����һ����y_temp������yֱ�Ӵ����Ҷ˺��������⸱���ã��Ҷ˺����п��ܻ��޸Ĵ����yֵ����
		f(t, dt,  y_temp.data(), dy_dt.data(), extern_data); //����dy_dt = f(t,y)
		for (int i = 0; i < y_number; i++)
			k1[i] = dt*dy_dt[i]; 

		//k2 = dt * f(t + dt / 2, y + k1 / 2),
		for (int i = 0; i < y_number; i++)
			y_temp[i] = y[i] + k1[i] / 2; //����y_temp = y+k1/2 
		f(t + dt / 2, dt,  y_temp.data(), dy_dt.data(), extern_data); //����dy_dt= f(t+dt/2, y+k1/2)
		for (int i = 0; i < y_number; i++)
			k2[i] = dt*dy_dt[i];

		//k3 = dt * f(t + dt / 2, y + k2 / 2),
		for (int i = 0; i < y_number; i++)
			y_temp[i] = y[i] + k2[i] / 2; //����y_temp = y+k2/2
		f(t + dt / 2, dt, y_temp.data(), dy_dt.data(), extern_data);//����dy_dt= f(t+dt/2, y+k2/2)
		for (int i = 0; i < y_number; i++)
			k3[i] = dt*dy_dt[i];

		//k4 = dt * f(t + dt, y + k3);
		for (int i = 0; i < y_number; i++)
			y_temp[i] = y[i] + k3[i];//����y_temp= y+ k3
		f(t + dt, dt, y_temp.data(), dy_dt.data(), extern_data); //����dy_dt= f(t+dt, y+k3)
		for (int i = 0; i < y_number; i++)
			k4[i] = dt*dy_dt[i];

		//y = y + (k1 + 2 * k2 + 2 * k3 + k4) / 6;
		for (int i = 0; i < y_number; i++)
			y[i] = y[i] + (k1[i] + 2*k2[i] + 2*k3[i] + k4[i])/6;

	}

};


void test_rk4();