
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


typedef void* aerodynamic_data;  //�������ݶ����ָ��

#ifdef __cplusplus 
extern "C"{
#endif 
	AERODYNAMIC_INTERP_API aerodynamic_data aerodynamic_open2(const char* aerodynamic_data_dir);
	AERODYNAMIC_INTERP_API void aerodynamic_close2(aerodynamic_data h);

	AERODYNAMIC_INTERP_API double GetS(aerodynamic_data h); //��ȡ�����������λΪ��ƽ��
	AERODYNAMIC_INTERP_API double GetL(aerodynamic_data h); //��ȡ��ͨ�ã��������ȣ���λΪ��
	AERODYNAMIC_INTERP_API double GetL_MX(aerodynamic_data h); //��ȡ��ת������������ȣ���չ������λΪ��

	AERODYNAMIC_INTERP_API double GetMaxAlpha(aerodynamic_data h); //�����е���󹥽ǣ���λΪ��
	AERODYNAMIC_INTERP_API double GetMinAlpha(aerodynamic_data h); //�����е���С���ǣ���λΪ��

	AERODYNAMIC_INTERP_API double GetMaxBeta(aerodynamic_data h); //�����е����໬�ǣ���λΪ��
	AERODYNAMIC_INTERP_API double GetMinBeta(aerodynamic_data h); //�����е���С�໬�ǣ���λΪ��


	/*********************************************************************************************************************/
	//�������Ի���Ϊ��λ��ϵ�к���
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

	//��������������ǡ��໬�ǣ� ��ֵ���ȶ���xcp�� ��������ֵģ��ڶ���������
	AERODYNAMIC_INTERP_API double GetXCP_Rad(aerodynamic_data h, double mach, double alpha, double beta);

	//��ȡ��ǰ����������໬�ǣ��£� �������ȶ�Ӧ�Ĺ���
	AERODYNAMIC_INTERP_API double GetMaxCLCD_Alpha_Rad(aerodynamic_data h, double mach, double beta);

	//���ݹ�����ƽ���ƫ�ǣ�
	AERODYNAMIC_INTERP_API double GetBalanceDeltaZ_Rad(aerodynamic_data h, double mach, double alpha, double beta);
	//���ݶ�ƫ����ȡƽ�⹥��
	AERODYNAMIC_INTERP_API double GetBalanceAlpha_Rad(aerodynamic_data h, double mach, double beta, double delta_z);
	//���ݸ�����ƽ�⹥�ǣ�������ϵ��
	AERODYNAMIC_INTERP_API double GetCYBalanceByAlpha_Rad(aerodynamic_data h, double mach, double alpha, double beta);
	//���ݸ�����ƽ������ϵ�������幥��
	AERODYNAMIC_INTERP_API double GetAlphaByCYBalance_Rad(aerodynamic_data h, double mach, double beta, double cy_balance);

	//���ݲ໬����ƽ���ƫ��
	AERODYNAMIC_INTERP_API double GetBalanceDeltaY_Rad(aerodynamic_data h, double mach, double alpha, double beta);
	//���ݶ�ƫ����ƽ��໬�ǣ�
	AERODYNAMIC_INTERP_API double GetBalanceBeta_Rad(aerodynamic_data h, double mach, double alpha, double delta_y);
	//���ݸ�����ƽ��໬�ǽǣ��������ϵ��
	AERODYNAMIC_INTERP_API double GetCZBalanceByBeta_Rad(aerodynamic_data h, double mach, double alpha, double beta);
	//���ݸ�����ƽ�������ϵ��������໬��
	AERODYNAMIC_INTERP_API double GetBetaByCZBalance_Rad(aerodynamic_data h, double mach, double alpha, double cz_balance);

	//���Ŭ˹����
	AERODYNAMIC_INTERP_API double GetMYWX0_Rad(aerodynamic_data h, double mach, double alpha, double beta);
	AERODYNAMIC_INTERP_API double GetMXWY0_Rad(aerodynamic_data h, double mach, double alpha, double beta);


	//������ֵ�����Ϊ��֧�ֶ���ϵ�����㣩
	//����ϵ�����������ضԹ��ǡ�������ƫ�ǵĵ���
	AERODYNAMIC_INTERP_API double GetCY_per_Alpha_Rad(aerodynamic_data h, double mach, double alpha, double beta);
	AERODYNAMIC_INTERP_API double GetCY_per_DeltaZ_Rad(aerodynamic_data h, double mach, double alpha, double beta, double delta_z);
	AERODYNAMIC_INTERP_API double GetMZ_per_Alpha_Rad(aerodynamic_data h, double mach, double alpha, double beta);
	AERODYNAMIC_INTERP_API double GetMZ_per_DeltaZ_Rad(aerodynamic_data h, double mach, double alpha, double beta, double delta_z);

	//������ϵ����ƫ�����ء����Ŭ˹���ضԲ໬�ǡ�ƫ����ƫ�ǵĵ���
	AERODYNAMIC_INTERP_API double GetCZ_per_Beta_Rad(aerodynamic_data h, double mach, double alpha, double beta);
	AERODYNAMIC_INTERP_API double GetCZ_per_DeltaY_Rad(aerodynamic_data h, double mach, double alpha, double beta, double delta_y);
	AERODYNAMIC_INTERP_API double GetMY_per_Beta_Rad(aerodynamic_data h, double mach, double alpha, double beta);
	AERODYNAMIC_INTERP_API double GetMY_per_DeltaY_Rad(aerodynamic_data h, double mach, double alpha, double beta, double delta_y);
	AERODYNAMIC_INTERP_API double GetMYWX0_per_Beta_Rad(aerodynamic_data h, double mach, double alpha, double beta);

	//��ת����ϵ���Թ�ת��ƫ�ǵĵ���
	AERODYNAMIC_INTERP_API double GetMX_per_DeltaX_Rad(aerodynamic_data h, double mach, double alpha, double beta, double delta_x);

#ifdef __cplusplus 
}
#endif 
