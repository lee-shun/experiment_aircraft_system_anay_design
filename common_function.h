// common_function.h

#pragma once
using namespace std;

string FormatFilePath(string filename, string output_dir, string postfix = "");
//{
//	if (postfix.length() != 0)
//		postfix = "_" + postfix;
//	return output_dir + filename + postfix + ".txt";
//}
//

string FormatDeltaRange(int delta_start, int delta_end);
//{
//		char delta_range[100];
//		sprintf(delta_range, "[%d,%d]", delta_start, delta_end);
//		return delta_range;
//}
