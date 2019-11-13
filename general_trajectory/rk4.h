#pragma once

//4阶龙格库塔积分
//用于对多元微分方程组进行积分，dy/dt = f(t,y)
//微分方程组（未知数）的个数为y_number, f(t,y)为右端函数
//参考资料：北理飞行力学：#2-9 导弹运动方程组的数值解法, 76页
//http://rosettacode.org/wiki/Runge-Kutta_method
//

//右端函数 , 比普通右端函数增加了一个步长（dt），因为某些控制规律的计算可能需要时间步长
typedef void (*RightFunction)(double t, double dt, double const y[], double dy_dt[], void* extern_data);

class rk4{
	int y_number = 0;//微分方程组的个数
	std::vector<double> k1, k2, k3, k4; 
	std::vector<double> dy_dt;// dy/dt
	std::vector<double> y_temp; //存储计算k的临时y值

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
			y_temp[i] = y[i]; //计算y_temp = y ，下一步将y_temp而不是y直接传入右端函数，避免副作用（右端函数中可能会修改传入的y值？）
		f(t, dt,  y_temp.data(), dy_dt.data(), extern_data); //计算dy_dt = f(t,y)
		for (int i = 0; i < y_number; i++)
			k1[i] = dt*dy_dt[i]; 

		//k2 = dt * f(t + dt / 2, y + k1 / 2),
		for (int i = 0; i < y_number; i++)
			y_temp[i] = y[i] + k1[i] / 2; //计算y_temp = y+k1/2 
		f(t + dt / 2, dt,  y_temp.data(), dy_dt.data(), extern_data); //计算dy_dt= f(t+dt/2, y+k1/2)
		for (int i = 0; i < y_number; i++)
			k2[i] = dt*dy_dt[i];

		//k3 = dt * f(t + dt / 2, y + k2 / 2),
		for (int i = 0; i < y_number; i++)
			y_temp[i] = y[i] + k2[i] / 2; //计算y_temp = y+k2/2
		f(t + dt / 2, dt, y_temp.data(), dy_dt.data(), extern_data);//计算dy_dt= f(t+dt/2, y+k2/2)
		for (int i = 0; i < y_number; i++)
			k3[i] = dt*dy_dt[i];

		//k4 = dt * f(t + dt, y + k3);
		for (int i = 0; i < y_number; i++)
			y_temp[i] = y[i] + k3[i];//计算y_temp= y+ k3
		f(t + dt, dt, y_temp.data(), dy_dt.data(), extern_data); //计算dy_dt= f(t+dt, y+k3)
		for (int i = 0; i < y_number; i++)
			k4[i] = dt*dy_dt[i];

		//y = y + (k1 + 2 * k2 + 2 * k3 + k4) / 6;
		for (int i = 0; i < y_number; i++)
			y[i] = y[i] + (k1[i] + 2*k2[i] + 2*k3[i] + k4[i])/6;

	}

};


void test_rk4();