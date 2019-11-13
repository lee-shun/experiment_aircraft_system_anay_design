#pragma once
#include "stdafx.h"
#include <windows.h>

//inline void prompt_exit(int exit_code) {
//	//printf("按回车键退出\n");
//	//getc(stdin);
//	//exit(exit_code);
//
//	if(exit_code !=0)
//		printf("发生错误，请截屏记录错误并联系技术支持。\n 关闭窗口以退出\n");
//	else 
//		printf("运行结束， 请关闭窗口以退出\n");
//	fflush(stdout);
//	//int c = getc(stdin);
//	Sleep(INFINITE);
//}
//

bool IsCommentLine(const char* line);

void split_file(const std::string filename, char delim, std::map<std::string, std::string>& name_values);

void split_file_pair_vector(const std::string filename, char delim, std::vector< std::pair<std::string, std::string> >& name_values) ;

//将字符串中的浮点数序列（例如 “ 1.1 1.2 1.3 1.4 ”）转换为浮点数组
std::vector<double> convert2doubles(std::string s);
