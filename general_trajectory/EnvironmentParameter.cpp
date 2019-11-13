#include "stdafx.h"
#include "EnvironmentParameter.h"
#include "..\utility.h"

using namespace std;

//void read2vector(const string& file_path, vector<double>& v){
//	ifstream fin(file_path.c_str());
//	//std::istream_
//	//std::istream_iterator<double> iit(fin);
//	//std::istream_iterator<double> end;
//	//std::copy(iit, end, std::back_inserter(v));
//
//	double value;
//	while (!fin.eof()) {
//		fin >> value;
//		v.push_back(value);
//	}
//}
//

//void EnvironmentParameter::Load(std::string path)
void EnvironmentParameter::Load()
{
	//! Todo 需要考虑处理path
	read2vector("ANDH.txt", ANDH);
	read2vector("ARHO.txt", ANDRHO);
	read2vector("ASonic.txt", ANDSonic);
}

