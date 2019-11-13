
// The following ifdef block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the AERODYNAMIC_INTERP_EXPORTS
// symbol defined on the command line. this symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// AERODYNAMIC_INTERP_API functions as being imported from a DLL, wheras this DLL sees symbols
// defined with this macro as being exported.
#ifdef AERODYNAMIC_INTERP_EXPORTS
#define AERODYNAMIC_INTERP_API __declspec(dllexport)
#else
#define AERODYNAMIC_INTERP_API __declspec(dllimport)
#endif


typedef void* aerodynamic_data;  //气动数据对象的指针

#ifdef __cplusplus 
extern "C"{
#endif 
	AERODYNAMIC_INTERP_API aerodynamic_data aerodynamic_open2(const char* aerodynamic_data_dir);
	AERODYNAMIC_INTERP_API void aerodynamic_close2(aerodynamic_data h);

	AERODYNAMIC_INTERP_API double GetS(aerodynamic_data h); //获取特征面积，单位为米平方
	AERODYNAMIC_INTERP_API double GetL(aerodynamic_data h); //获取（通用）特征长度，单位为米
	AERODYNAMIC_INTERP_API double GetL_MX(aerodynamic_data h); //获取滚转方向的特征长度（翼展），单位为米

	AERODYNAMIC_INTERP_API double GetMaxAlpha(aerodynamic_data h); //工况中的最大攻角，单位为度
	AERODYNAMIC_INTERP_API double GetMinAlpha(aerodynamic_data h); //工况中的最小攻角，单位为度

	AERODYNAMIC_INTERP_API double GetMaxBeta(aerodynamic_data h); //工况中的最大侧滑角，单位为度
	AERODYNAMIC_INTERP_API double GetMinBeta(aerodynamic_data h); //工况中的最小侧滑角，单位为度


	/*********************************************************************************************************************/
	//下面是以弧度为单位的系列函数
	/*********************************************************************************************************************/


	AERODYNAMIC_INTERP_API double GetCX0_Rad(aerodynamic_data h, double mach, double alpha, double beta);
	AERODYNAMIC_INTERP_API double GetCX_by_Delta_Rad(aerodynamic_data h, double mach, double alpha, double beta, double delta_x, double delta_y, double delta_z);
	AERODYNAMIC_INTERP_API double GetCX_Rad(aerodynamic_data h, double mach, double alpha, double beta, double delta_x, double delta_y, double delta_z);

	AERODYNAMIC_INTERP_API double GetMX0_Rad(aerodynamic_data h, double mach, double alpha, double beta);
	AERODYNAMIC_INTERP_API double GetMX_by_DeltaX_Rad(aerodynamic_data h, double mach, double alpha, double beta, double delta_x);
	AERODYNAMIC_INTERP_API double GetMX_Rad(aerodynamic_data h, double mach, double alpha, double beta, double delta_x);

	AERODYNAMIC_INTERP_API double GetMXWX0_Rad(aerodynamic_data h, double mach, double alpha, double beta);

	AERODYNAMIC_INTERP_API double GetCY0_Rad(aerodynamic_data h, double mach, double alpha, double beta=0);
	AERODYNAMIC_INTERP_API double GetCY_by_Delta_Rad(aerodynamic_data h, double mach, double alpha, double beta, double delta_x, double delta_y, double delta_z);
	AERODYNAMIC_INTERP_API double GetCY_Rad(aerodynamic_data h, double mach, double alpha, double beta, double delta_x, double delta_y, double delta_z);

	AERODYNAMIC_INTERP_API double GetMZ0_Rad(aerodynamic_data h, double mach, double alpha, double beta);
	AERODYNAMIC_INTERP_API double GetMZ_by_DeltaZ_Rad(aerodynamic_data h, double mach, double alpha, double beta, double delta_z);
	AERODYNAMIC_INTERP_API double GetMZ_Rad(aerodynamic_data h, double mach, double alpha, double beta, double delta_z);

	AERODYNAMIC_INTERP_API double GetMZWZ0_Rad(aerodynamic_data h, double mach, double alpha, double beta);

	AERODYNAMIC_INTERP_API double GetCZ0_Rad(aerodynamic_data h, double mach, double alpha, double beta);
	AERODYNAMIC_INTERP_API double GetCZ_by_Delta_Rad(aerodynamic_data h, double mach, double alpha, double beta, double delta_x, double delta_y, double delta_z);
	AERODYNAMIC_INTERP_API double GetCZ_Rad(aerodynamic_data h, double mach, double alpha, double beta, double delta_x, double delta_y, double delta_z);

	AERODYNAMIC_INTERP_API double GetMY0_Rad(aerodynamic_data h, double mach, double alpha, double beta);
	AERODYNAMIC_INTERP_API double GetMY_by_DeltaY_Rad(aerodynamic_data h, double mach, double alpha, double beta, double delta_y);
	AERODYNAMIC_INTERP_API double GetMY_Rad(aerodynamic_data h, double mach, double alpha, double beta, double delta_y);

	AERODYNAMIC_INTERP_API double GetMYWY0_Rad(aerodynamic_data h, double mach, double alpha, double beta);

	//根据马赫数、攻角、侧滑角， 插值静稳定度xcp， 在气动插值模块第二版中新增
	AERODYNAMIC_INTERP_API double GetXCP_Rad(aerodynamic_data h, double mach, double alpha, double beta);

	//求取当前马赫数（及侧滑角）下， 最佳升阻比对应的攻角
	AERODYNAMIC_INTERP_API double GetMaxCLCD_Alpha_Rad(aerodynamic_data h, double mach, double beta);

	//根据攻角求平衡舵偏角，
	AERODYNAMIC_INTERP_API double GetBalanceDeltaZ_Rad(aerodynamic_data h, double mach, double alpha, double beta);
	//根据舵偏角求取平衡攻角
	AERODYNAMIC_INTERP_API double GetBalanceAlpha_Rad(aerodynamic_data h, double mach, double beta, double delta_z);
	//根据给定的平衡攻角，求升力系数
	AERODYNAMIC_INTERP_API double GetCYBalanceByAlpha_Rad(aerodynamic_data h, double mach, double alpha, double beta);
	//根据给定的平衡升力系数，反插攻角
	AERODYNAMIC_INTERP_API double GetAlphaByCYBalance_Rad(aerodynamic_data h, double mach, double beta, double cy_balance);

	//根据侧滑角求平衡舵偏角
	AERODYNAMIC_INTERP_API double GetBalanceDeltaY_Rad(aerodynamic_data h, double mach, double alpha, double beta);
	//根据舵偏角求平衡侧滑角，
	AERODYNAMIC_INTERP_API double GetBalanceBeta_Rad(aerodynamic_data h, double mach, double alpha, double delta_y);
	//根据给定的平衡侧滑角角，求侧向力系数
	AERODYNAMIC_INTERP_API double GetCZBalanceByBeta_Rad(aerodynamic_data h, double mach, double alpha, double beta);
	//根据给定的平衡侧向力系数，反插侧滑角
	AERODYNAMIC_INTERP_API double GetBetaByCZBalance_Rad(aerodynamic_data h, double mach, double alpha, double cz_balance);

	//马格努斯力矩
	AERODYNAMIC_INTERP_API double GetMYWX0_Rad(aerodynamic_data h, double mach, double alpha, double beta);
	AERODYNAMIC_INTERP_API double GetMXWY0_Rad(aerodynamic_data h, double mach, double alpha, double beta);


	//计算各种导数（为了支持动力系数计算）
	//升力系数、俯仰力矩对攻角、俯仰舵偏角的导数
	AERODYNAMIC_INTERP_API double GetCY_per_Alpha_Rad(aerodynamic_data h, double mach, double alpha, double beta);
	AERODYNAMIC_INTERP_API double GetCY_per_DeltaZ_Rad(aerodynamic_data h, double mach, double alpha, double beta, double delta_z);
	AERODYNAMIC_INTERP_API double GetMZ_per_Alpha_Rad(aerodynamic_data h, double mach, double alpha, double beta);
	AERODYNAMIC_INTERP_API double GetMZ_per_DeltaZ_Rad(aerodynamic_data h, double mach, double alpha, double beta, double delta_z);

	//侧向力系数、偏航力矩、马格努斯力矩对侧滑角、偏航舵偏角的导数
	AERODYNAMIC_INTERP_API double GetCZ_per_Beta_Rad(aerodynamic_data h, double mach, double alpha, double beta);
	AERODYNAMIC_INTERP_API double GetCZ_per_DeltaY_Rad(aerodynamic_data h, double mach, double alpha, double beta, double delta_y);
	AERODYNAMIC_INTERP_API double GetMY_per_Beta_Rad(aerodynamic_data h, double mach, double alpha, double beta);
	AERODYNAMIC_INTERP_API double GetMY_per_DeltaY_Rad(aerodynamic_data h, double mach, double alpha, double beta, double delta_y);
	AERODYNAMIC_INTERP_API double GetMYWX0_per_Beta_Rad(aerodynamic_data h, double mach, double alpha, double beta);

	//滚转力矩系数对滚转舵偏角的导数
	AERODYNAMIC_INTERP_API double GetMX_per_DeltaX_Rad(aerodynamic_data h, double mach, double alpha, double beta, double delta_x);

#ifdef __cplusplus 
}
#endif 
