#pragma once
#include "stdafx.h"
#include <windows.h>

//inline void prompt_exit(int exit_code) {
//	//printf("���س����˳�\n");
//	//getc(stdin);
//	//exit(exit_code);
//
//	if(exit_code !=0)
//		printf("���������������¼������ϵ����֧�֡�\n �رմ������˳�\n");
//	else 
//		printf("���н����� ��رմ������˳�\n");
//	fflush(stdout);
//	//int c = getc(stdin);
//	Sleep(INFINITE);
//}
//

bool IsCommentLine(const char* line);

void split_file(const std::string filename, char delim, std::map<std::string, std::string>& name_values);

void split_file_pair_vector(const std::string filename, char delim, std::vector< std::pair<std::string, std::string> >& name_values) ;

//���ַ����еĸ��������У����� �� 1.1 1.2 1.3 1.4 ����ת��Ϊ��������
std::vector<double> convert2doubles(std::string s);
