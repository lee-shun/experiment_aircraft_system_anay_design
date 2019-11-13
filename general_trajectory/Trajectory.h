#pragma once

#include "aerodynamic_model.h"
#include "EnvironmentParameter.h"
#include "rotation_inertia_model.h"


extern EnvironmentParameter g_atmosphere_model; 
class trajectory{
public:
	trajectory(){
		//vs 2015��bug�������еı�����ʼ�����ᱻ����
		_aero_model = NULL;
		_rotation_inertia = NULL;
	}
	virtual ~trajectory(){};

	RotationInertiaModel* _rotation_inertia = NULL; //ת������ģ��

	AerodynamicModel*	_aero_model = NULL; //����ģ��


public:
	static void trajectory_equations(double t, double dt, double const y[], double dy_dt[], void* extern_data){
		trajectory* p = (trajectory*)extern_data;
		p->trajectory_equations_internal(t, dt, y, dy_dt);
	}

private:

	virtual void trajectory_equations_internal(double t, double dt, double const y[], double dy_dt[]) = 0;
	virtual int get_number_of_runge_kutta() = 0;
	virtual void no_differential_calculus_equations(double t, double dt) = 0;

};