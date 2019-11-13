#include "stdafx.h"
#include "utils.h"
#include "const.h"
#include <boost/algorithm/string.hpp>
#include "../utility.h"
using namespace std;
using namespace boost::algorithm;

bool IsCommentLine(const char* line) {
	return line[0] == '#'; //#��ͷΪע����
}

bool split(const std::string &s, char delim, std::string& first, std::string& second) {
	std::string::size_type pos = s.find_first_of(delim);
	if (pos == std::string::npos)
		return false;

	first = s.substr(0, pos);
	second = s.substr(pos + 1, s.length() - pos);
	return true;
}

//�ļ���ʽΪ�� name=value ��ÿ�У�
//void split_file(const std::string filename, char delim, std::map<std::string, std::string>& name_values) {
void split_file_pair_vector(const std::string filename, char delim, std::vector< std::pair<std::string, std::string> >& name_values) {
	char line[MAX_LINE] = "";
	//ifstream fin(filename.c_str());
	//fin.exceptions(ifstream::failbit | ifstream::badbit);
	FILE* fin = fopen(filename.c_str(), "r");
	if (fin == NULL) {
		printf("�޷����ļ�: %s", filename.c_str());
		prompt_exit(1);
	}

	int row = 0;
	//while (!fin.eof()) {
	while (!feof(fin)) {
		//fin.getline(line, MAX_LINE);
		if (0 != fgets(line, MAX_LINE, fin))
		{
			if (line[strlen(line) - 1] == '\n')
				line[strlen(line) - 1] = 0;
			row++;

			if (IsCommentLine(line)) {
				continue;  //����ע����
			}
			else {  //��ע����
				std::string s_line = line;
				boost::trim(s_line);

				if (!s_line.empty()) { //�ǿ���
					std::string name;
					std::string value;

					//if (!split(line, '=', name, value))
					if (!split(line, delim, name, value))
					{
						printf("�����ļ���%s����%d�г���\n", filename.c_str(), row);
						prompt_exit(1);
					}

					trim(name);
					trim(value);
					
					name_values.push_back({ name,value });
				}

			}
		}
	}

	fclose(fin);
}


//�ļ���ʽΪ�� name=value ��ÿ�У�
void split_file(const std::string filename, char delim, std::map<std::string, std::string>& name_values) {
	char line[MAX_LINE] = "";
	//ifstream fin(filename.c_str());
	//fin.exceptions(ifstream::failbit | ifstream::badbit);
	FILE* fin = fopen(filename.c_str(), "r");
	if (fin == NULL) {
		printf("�޷����ļ�: %s", filename.c_str());
		prompt_exit(1);
	}

	int row = 0;
	//while (!fin.eof()) {
	while (!feof(fin)) {
		//fin.getline(line, MAX_LINE);
		if (0 != fgets(line, MAX_LINE, fin))
		{
			if (line[strlen(line) - 1] == '\n')
				line[strlen(line) - 1] = 0;
			row++;

			if (IsCommentLine(line)) {
				continue;  //����ע����
			}
			else {  //��ע����
				std::string s_line = line;
				boost::trim(s_line);

				if (!s_line.empty()) { //�ǿ���
					std::string name;
					std::string value;

					//if (!split(line, '=', name, value))
					if (!split(line, delim, name, value))
					{
						printf("�����ļ���%s����%d�г���\n", filename.c_str(), row);
						prompt_exit(1);
					}

					//trim_right(name);
					//trim_left(value);
					trim(name);
					trim(value);
					name_values[name] = value;
				}

			}
		}
	}

	fclose(fin);
}

vector<double> convert2doubles(std::string s) {
	vector<double> ret;
	istringstream iss(s);
	double value;
	while (iss >> value) {
		ret.push_back(value);
	}
	return ret;
}
