// utility.cpp
#include "stdafx.h"
#include <string>
#include <vector>
#include <windows.h>
#include <iostream>

#ifdef WIN32
#include <direct.h>
#define GetCurrentDir _getcwd
#else
#include <unistd.h>
#define GetCurrentDir getcwd
#endif

using namespace std;
#include "utility.h"

bool g_exit_keyborad = true;
//string GetCurrentPath(){
//	char cCurrentPath[FILENAME_MAX];
//
//	if (!GetCurrentDir(cCurrentPath, sizeof(cCurrentPath)))
//	{
//		throw errno;
//	}
//
//	cCurrentPath[sizeof(cCurrentPath) - 1] = '\0'; /* not really required */
//
//	//printf("The current working directory is %s", cCurrentPath);
//	return cCurrentPath;
//}
//
//bool dirExists(const std::string& dirName_in)
//{
//	DWORD ftyp = GetFileAttributesA(dirName_in.c_str());
//	if (ftyp == INVALID_FILE_ATTRIBUTES)
//		return false;  //something is wrong with your path!
//
//	if (ftyp & FILE_ATTRIBUTE_DIRECTORY)
//		return true;   // this is a directory!
//
//	return false;    // this is not a directory!
//}
//
//BOOL AnsiToUnicode16(CHAR *in_Src, WCHAR *out_Dst, INT in_MaxLen)
//{
//	/* locals */
//	INT lv_Len;
//
//	// do NOT decrease maxlen for the eos
//	if (in_MaxLen <= 0)
//		return FALSE;
//
//	// let windows find out the meaning of ansi
//	// - the SrcLen=-1 triggers MBTWC to add a eos to Dst and fails if MaxLen is too small.
//	// - if SrcLen is specified then no eos is added
//	// - if (SrcLen+1) is specified then the eos IS added
//	lv_Len = MultiByteToWideChar(CP_ACP, 0, in_Src, -1, out_Dst, in_MaxLen);
//
//	// validate
//	if (lv_Len < 0)
//		lv_Len = 0;
//
//	// ensure eos, watch out for a full buffersize
//	// - if the buffer is full without an eos then clear the output like MBTWC does
//	//   in case of too small outputbuffer
//	// - unfortunately there is no way to let MBTWC return shortened strings,
//	//   if the outputbuffer is too small then it fails completely
//	if (lv_Len < in_MaxLen)
//		out_Dst[lv_Len] = 0;
//	else if (out_Dst[in_MaxLen - 1])
//		out_Dst[0] = 0;
//
//	// done
//	return TRUE;
//}
//
//string MultiByteToUTF8(char* str){
//	WCHAR wstr[MAX_PATH + 1];
//	AnsiToUnicode16(str, wstr, MAX_PATH);
//	return  utf8util::UTF8FromUTF16(wstr);
//
//}
//
//void read2vector(const string& file_path, vector<double>& v){
//	ifstream fin(file_path.c_str());
//	std::istream_iterator<double> iit(fin);
//	std::istream_iterator<double> end;
//	std::copy(iit, end, std::back_inserter(v));
//}
//
string g_filename = "";

void read2vector(const string& file_path, vector<double>& v) {

	g_filename = file_path;
	ifstream fin(file_path.c_str());
	std::istream_iterator<double> iit(fin);
	std::istream_iterator<double> end;
	std::copy(iit, end, std::back_inserter(v));
}