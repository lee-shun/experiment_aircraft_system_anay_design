// Interp.h
// 存放一些公共的插值模板函数

#pragma  once

#include <vector>
using namespace std;

//线性插值
//double interp_linear(double y1, double y2, double x1, double x2, double x);

//线性插值
inline double interp_linear(double y1, double y2, double x1, double x2, double x){
	_ASSERT((x2 - x1) != 0);
	if ((x2 - x1) == 0) {
		throw std::exception(" x1 = x2 , interp_linear failed\n");
	}
	double k = (y2 - y1) / (x2 - x1);//斜率k
	return y1 + k*(x - x1);
}



//求插值索引
//要求值数组从小到大排列
template<typename Array_T>
int find_interp_index_asc(Array_T& v, int length,  double x){
	//如果x大于数组最后一个值，那么插值索引为倒数第二个
	if (x >= v[length-1])
		return length - 2;
	
	//找到一个index，使得 v[index] < x <= v[index+1]
	int index = 0;
	while (x > v[index + 1])
		index++;

	return index;
}

//求插值索引
//要求值数组从大到小排列
template<typename Array_T>
int find_interp_index_dsc(Array_T& v, int length, double x){
	//如果x大于数组第一个值，那么插值索引为0
	if (x >= v[0])
		return 0;

	//找到一个index，使得v[index] >= x > v[index+1]
	int index = length - 2;
	while(x > v[index])
		index--;

	return index;
}

//求插值索引
//不对数组从大到小或者从小到大做要求
template<typename Array_T>
int find_interp_index(Array_T& v, int length, double x){

	if (v[0] >= v[length - 1]){ //数组从大到小排列
		return find_interp_index_dsc(v, length, x);
	}
	else{//从小到大排列
		return find_interp_index_asc(v, length, x);
	}

	
}




//int find_interp_index(vector<double> v, double x){
//	//如果x大于数组最后一个值，那么插值索引为倒数第二个
//	if (x > v[v.size()-1])
//		return v.size() - 2;
//	
//	//找到一个index，使得 v[index] < x < v[index+1]
//	int index = 0;
//	while (x > v[index + 1])
//		index++;
//
//	return index;
//}


//int find_interp_index_1(vector<double>& v, double x){
//	//无法替换成std::begin，因为c++ 98不支持
//	vector<double>::iterator it = find_if(v.begin(), v.end(), std::bind1st(std::less_equal<double>() , x));
//	if (it == v.end())
//		return v.size() - 2;
//
//	int index = it - v.begin() -1;
//	if (index < 0) index = 0;
//
//	return index;
//}

//单变元线性插值
//values是值域数组，variables是自变量数组，
template<typename Array_T>
double interp11(Array_T& Y, const vector<double>&X, double x, bool reverse=false){
	if (X.size() == 1)  // 如果只有一组工况
		return Y[0];
	
	if (reverse){//反插， 例如通过升力系数，反插攻角
		int index = find_interp_index(Y, Y.size(), x);
		return interp_linear(X[index], X[index + 1], Y[index], Y[index + 1], x);
	}
	else{//正常插值
		int index = find_interp_index(X, X.size(), x);
		return interp_linear(Y[index], Y[index + 1], X[index], X[index + 1], x);
	}
}


//双变元线性插值
template<typename Array2D_T>
double interp12( Array2D_T& array2d, const vector<double>& array_x, const vector<double>& array_y, double x, double y, bool reverse=false){
	if (array_x.size() == 1)  // 如果只有一组工况
		return interp11(array2d[0], array_y, y, reverse);
	
	//正常插值
	int index = find_interp_index(array_x,array_x.size(), x);
	double v0 = interp11(array2d[index], array_y, y, reverse);
	double v1 = interp11(array2d[index+1], array_y, y, reverse);
	return interp_linear(v0, v1, array_x[index], array_x[index + 1], x);

}

//三变元线性插值
template<typename Array3D_T>
double interp13( Array3D_T& array3d, const vector<double>& array_x, const vector<double>& array_y, const vector<double>& array_z,double x, double y, double z, bool reverse=false){
	if (array_x.size() == 1)  // 如果只有一组工况
		return interp12(array3d[0], array_y, array_z, y, z, reverse);
	
	//正常插值
	int index = find_interp_index(array_x, array_x.size(), x);
	double v1 = interp12(array3d[index], array_y, array_z, y, z, reverse);
	double v2 = interp12(array3d[index+1], array_y, array_z, y, z, reverse);
	return interp_linear(v1, v2, array_x[index], array_x[index + 1], x);
}


