// Interp.h
// ���һЩ�����Ĳ�ֵģ�庯��

#pragma  once

#include <vector>
using namespace std;

//���Բ�ֵ
//double interp_linear(double y1, double y2, double x1, double x2, double x);

//���Բ�ֵ
inline double interp_linear(double y1, double y2, double x1, double x2, double x){
	_ASSERT((x2 - x1) != 0);
	if ((x2 - x1) == 0) {
		throw std::exception(" x1 = x2 , interp_linear failed\n");
	}
	double k = (y2 - y1) / (x2 - x1);//б��k
	return y1 + k*(x - x1);
}



//���ֵ����
//Ҫ��ֵ�����С��������
template<typename Array_T>
int find_interp_index_asc(Array_T& v, int length,  double x){
	//���x�����������һ��ֵ����ô��ֵ����Ϊ�����ڶ���
	if (x >= v[length-1])
		return length - 2;
	
	//�ҵ�һ��index��ʹ�� v[index] < x <= v[index+1]
	int index = 0;
	while (x > v[index + 1])
		index++;

	return index;
}

//���ֵ����
//Ҫ��ֵ����Ӵ�С����
template<typename Array_T>
int find_interp_index_dsc(Array_T& v, int length, double x){
	//���x���������һ��ֵ����ô��ֵ����Ϊ0
	if (x >= v[0])
		return 0;

	//�ҵ�һ��index��ʹ��v[index] >= x > v[index+1]
	int index = length - 2;
	while(x > v[index])
		index--;

	return index;
}

//���ֵ����
//��������Ӵ�С���ߴ�С������Ҫ��
template<typename Array_T>
int find_interp_index(Array_T& v, int length, double x){

	if (v[0] >= v[length - 1]){ //����Ӵ�С����
		return find_interp_index_dsc(v, length, x);
	}
	else{//��С��������
		return find_interp_index_asc(v, length, x);
	}

	
}




//int find_interp_index(vector<double> v, double x){
//	//���x�����������һ��ֵ����ô��ֵ����Ϊ�����ڶ���
//	if (x > v[v.size()-1])
//		return v.size() - 2;
//	
//	//�ҵ�һ��index��ʹ�� v[index] < x < v[index+1]
//	int index = 0;
//	while (x > v[index + 1])
//		index++;
//
//	return index;
//}


//int find_interp_index_1(vector<double>& v, double x){
//	//�޷��滻��std::begin����Ϊc++ 98��֧��
//	vector<double>::iterator it = find_if(v.begin(), v.end(), std::bind1st(std::less_equal<double>() , x));
//	if (it == v.end())
//		return v.size() - 2;
//
//	int index = it - v.begin() -1;
//	if (index < 0) index = 0;
//
//	return index;
//}

//����Ԫ���Բ�ֵ
//values��ֵ�����飬variables���Ա������飬
template<typename Array_T>
double interp11(Array_T& Y, const vector<double>&X, double x, bool reverse=false){
	if (X.size() == 1)  // ���ֻ��һ�鹤��
		return Y[0];
	
	if (reverse){//���壬 ����ͨ������ϵ�������幥��
		int index = find_interp_index(Y, Y.size(), x);
		return interp_linear(X[index], X[index + 1], Y[index], Y[index + 1], x);
	}
	else{//������ֵ
		int index = find_interp_index(X, X.size(), x);
		return interp_linear(Y[index], Y[index + 1], X[index], X[index + 1], x);
	}
}


//˫��Ԫ���Բ�ֵ
template<typename Array2D_T>
double interp12( Array2D_T& array2d, const vector<double>& array_x, const vector<double>& array_y, double x, double y, bool reverse=false){
	if (array_x.size() == 1)  // ���ֻ��һ�鹤��
		return interp11(array2d[0], array_y, y, reverse);
	
	//������ֵ
	int index = find_interp_index(array_x,array_x.size(), x);
	double v0 = interp11(array2d[index], array_y, y, reverse);
	double v1 = interp11(array2d[index+1], array_y, y, reverse);
	return interp_linear(v0, v1, array_x[index], array_x[index + 1], x);

}

//����Ԫ���Բ�ֵ
template<typename Array3D_T>
double interp13( Array3D_T& array3d, const vector<double>& array_x, const vector<double>& array_y, const vector<double>& array_z,double x, double y, double z, bool reverse=false){
	if (array_x.size() == 1)  // ���ֻ��һ�鹤��
		return interp12(array3d[0], array_y, array_z, y, z, reverse);
	
	//������ֵ
	int index = find_interp_index(array_x, array_x.size(), x);
	double v1 = interp12(array3d[index], array_y, array_z, y, z, reverse);
	double v2 = interp12(array3d[index+1], array_y, array_z, y, z, reverse);
	return interp_linear(v1, v2, array_x[index], array_x[index + 1], x);
}


