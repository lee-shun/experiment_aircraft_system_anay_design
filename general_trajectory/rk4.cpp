#include "stdafx.h"
#include "rk4.h"

void rate(double t, double dt, double const y[], double dy_dt[], void* extern_data){
	dy_dt[0] = t*sqrt(y[0]);
}
void test_rk4(){
	double y[1], x, y2;
	double x0 = 0, x1 = 10, dx = .1;
	int i; 
	int n = (int)(1 + (x1 - x0) / dx);
	y[0] = 1;

	rk4 rk(1);
	for (i = 0; i < n; i += 1) {
		x = x0 + dx * i;
		y2 = pow(x * x / 4 + 1, 2);
		printf("%g\t%g\t%g\n", x, y[0], y[0] / y2 - 1);
		rk.run(rate, dx, x, y, NULL);
	}
}

