#include "stdafx.h"
#include "rk4.h"
//#include "..\AeroInterp\aerodynamic_interp\aerodynamic_interp.h"
#include "six_degree_trajectory_without_control.h"
#include "three_degree_trajectory.h"
#include "event.h"
#include "event_manager.h"
#include "utils.h"
#include "../utility.h"

using namespace std;

char*  g_file_to_read = "";
double g_step = 0.01;

//double g_platform_altitude = 0;//ƽ̨���Σ� ������Ŀ�ꡢ�Ƶ�վ��Y���꣬���������������

//���ػ��ֲ���
//double read_step(){
//	g_file_to_read = "step.txt";
//	ifstream fin(g_file_to_read);
//	fin.exceptions(ifstream::failbit | ifstream::badbit);
//	double h;
//	fin >> h;
//	return h;
//}

void read_missile_type(int* flag_type_, int* flag_channel_){
	g_file_to_read = "missile.type";
	ifstream fin(g_file_to_read);
	fin.exceptions(ifstream::failbit | ifstream::badbit);
	fin >> *flag_type_;
	if (*flag_type_ == 1)
	{
		*flag_channel_ = 0;
	}
	else
	{
		fin >> *flag_channel_;
	}
}

//��ȡ��������·�� 
//string get_environment_parameter_path(){
//	g_file_to_read = "enviroment_parameter_path.txt";
//	ifstream fin(g_file_to_read);
//	fin.exceptions(ifstream::failbit | ifstream::badbit);
//
//	char path[MAX_PATH];
//	fin.getline(path, MAX_PATH);
//	return path;
//}


//����ģ��
EnvironmentParameter g_atmosphere_model; 

void Load_Time_Alpha(const char* filename, vector<double>& ts, vector<double>& alphas) {
	FILE* fin = fopen(filename, "r");
	char line[1024];
	fgets(line, 1024, fin);
	double t, alpha;
	while (!feof(fin)) {
		fscanf(fin, "%lf", &t);
		for (int i = 1; i < 10; i++)
			fscanf(fin, "%s", &line);
		fscanf(fin, "%lf", &alpha);
		ts.push_back(t);
		alphas.push_back(alpha/RAD);
		fgets(line, 1024, fin);
	}
}

void print_current_dir() {
	char pwd[MAX_PATH];
	GetCurrentDirectoryA(MAX_PATH, pwd);

	printf("current dir: %s\n", pwd);
}

double g_qianzhi_alpha_limit = 10;
double g_qianzhi_beta_limit = 10;
void read_qianzhi_alpha_beta_range() {
	g_file_to_read = "threepoint_alpha_beta_limit.cfg";
	ifstream fin(g_file_to_read);
	if (!fin)
		return;
	fin.exceptions(ifstream::failbit | ifstream::badbit);
	fin >> g_qianzhi_alpha_limit >> g_qianzhi_beta_limit;
}

int g_qianzhi_ajust_flag = 1;
void read_qianzhi_ajust() {
	g_file_to_read = "threepoint_ajust.cfg";
	ifstream fin(g_file_to_read);
	if (!fin)
		return;
	fin.exceptions(ifstream::failbit | ifstream::badbit);
	fin >> g_qianzhi_ajust_flag ;
}

int _tmain(int argc, _TCHAR* argv[])
{
	print_current_dir();
	//test_rk4();


	int wait_time = 10;
	if (argc == 2)
		wait_time = _tstoi(argv[1]);

	try{
		g_atmosphere_model.Load();
		read_qianzhi_alpha_beta_range();
		read_qianzhi_ajust();


		//six_degree_trajectory_without_control tj;
		//tj.states[six_degree_trajectory_without_control::V_Index] = 250;
		//tj.states[six_degree_trajectory_without_control::Y_Index] = 12000;
		//tj.states[six_degree_trajectory_without_control::Mass_Index] = 560;
		three_degree_trajectory tj;
		

		read_missile_type(&tj.flag_type, &tj.flag_channel);
		g_step = 0.01;
		double t = 0;
#if 0
		tj._states[three_degree_trajectory::V_Index] = 250;
		tj._states[three_degree_trajectory::Y_Index] = 12000;
		tj._states[three_degree_trajectory::Mass_Index] = 560;

		tj.Jx = 10.6;
		tj.Jy = 247.5;
		tj.Jz = 248.7;

		//tj.aero_data = aerodynamic_open("C:\\Users\\liwei\\Desktop\\wcmd��������\\������������������");
		NormalAerodynamicModel aero_model("C:\\Users\\liwei\\Desktop\\wcmd��������\\������������������");
		tj._aero_model = &aero_model;
		FixedMassModel mass_model(0);
		tj._mass_model = &mass_model;
		ZeorThrustModel thrust_model;
		tj._thrust_model = &thrust_model;
#endif

#if 0
		//DeltaYZ_ControlModel control_model(t);
		//control_model.delta_y_t_.push_back(0);
		//control_model.delta_y_.push_back(0);
		//control_model.delta_z_t_.push_back(0);
		//control_model.delta_z_.push_back(0);

		//AlphaBetaGammaV_ControlModel control_model(t);
		//control_model.gammav_t_array.push_back(0);
		//control_model.gammav_array.push_back(0);
		//tj._states[three_degree_trajectory::Theta_Index] = -53.1340159915 / RAD;
		////Load_T_Alpha("result_states - (�޼����޵���).txt", control_model.alpha_t_array, control_model.alpha_array);
		//Load_T_Alpha("result_states - 12000�߶Ⱦ���60����0(�޼����޵���).txt", control_model.alpha_t_array, control_model.alpha_array);
		////Load_T_Alpha("result_states - 12000�߶Ⱦ���60����0�����ַ��������С��1%��.txt", control_model.alpha_t_array, control_model.alpha_array);
		////control_model.alpha_t_array.push_back(0);
		////control_model.alpha_array.push_back(10/RAD);
		//control_model.beta_t_array.push_back(0);
		//control_model.beta_array.push_back(0);

		//Y_ControlModel control_model(t);
		//control_model.y_t_array.push_back(0);
		//control_model.y_array.push_back(12000);
		//control_model.y_t_array.push_back(60);
		//control_model.y_array.push_back(0);

		Proportional_Navigation_ControlModel control_model(t, 4, 13000, 0, 1000);
		tj._ideal_control_model = &control_model;
#endif
		//�¼����������������п��ܷ������¼�
		EventManager em;
		em.Load("time_sequence.txt");
		em.Process(t, &tj);
		//���˵�����ʼ�����

		//���������ʼ״̬
		tj.no_differential_calculus_equations(t, g_step); //����ǻ��ַ��̶�Ӧ��״̬����������������ƹ��ɣ����㵱ǰʱ���Ӧ���������ֵ��states�����������ܵĵ���״̬��� 

		FILE* f_result = fopen("result_states.txt", "w");
		FILE* f_donglixishu = fopen("donglixishu.txt", "w");
		FILE* f_donglixishu_new = fopen("donglixishu_new.txt", "w");
		FILE* f_result2 = fopen("result_states2.txt", "w");
		FILE* f_focus_motion_law = fopen("focus_motion_law.txt", "w");

		FILE* f_zhixin = fopen("zhixin.txt", "w");
		FILE* f_q= fopen("target_info.txt", "w");
		tj.output_title(f_result, f_donglixishu, f_donglixishu_new, f_q);
		tj.result(f_result, f_donglixishu, f_donglixishu_new, t); //�����ǰ������״̬
		tj.output_result2(f_result2, t);
		tj.output_title_focus_motion_law(f_focus_motion_law);
		tj.output_focus_motion_law(f_focus_motion_law, t);
		tj.output_q(f_q, t);
		//tj.output_zhixin(f_zhixin, t);
		//��ʼ�����������������
		rk4 rk(tj.get_number_of_runge_kutta());

		//��ʼ�����������
		//do{
		while (1) {
			//if (tj.near_end())
				//step = 0.001;

			//�� t -->  t+step
			rk.run(trajectory::trajectory_equations, g_step, t, tj._states, &tj); //���ݵ�ǰʱ��(t)��states��ȡ��һ��ʱ��(t+step)��states

			tj.no_differential_calculus_equations(t + g_step, g_step); //����ǻ��ַ��̶�Ӧ��״̬������������ƹ��ɣ����㵱ǰʱ���Ӧ���������ֵ��states�����������ܵĵ���״̬��� 			//�����ʵ㵯����˵��delta_y,delta_z, alpha, beta, gamma_v ��ֵ�� �ڵ��Ĵ��Ҷ��Ӻ��������У�����ݴ����t+hֵ������������ƹ����������㣬��˴���Ҳ���Բ����ã��������ʱ�����Եõ���ȷ��ֵ�����������뷨����ش���֯����ʱ�������޵ĸ�����Ǹ������ڿӣ�����ֶ��ĵ���״̬��������������Ӧ��״̬��

			if (tj.is_end(t+g_step))
				break;

			//�������״̬
			tj.result(f_result, f_donglixishu, f_donglixishu_new, t + g_step);
			tj.output_result2(f_result2, t + g_step);	
			tj.output_zhixin(f_zhixin, t + g_step);
			tj.output_focus_motion_law(f_focus_motion_law, t + g_step);
			tj.output_q(f_q, t);
			t = t + g_step;

			em.Process(t, &tj); //ʱ�������ʱ�䵽�ˣ���ٷ���Ӧ���¼�
		}
		//} while (!tj.is_end(t));
		

		fclose(f_result);
		fclose(f_donglixishu);
		fclose(f_donglixishu_new);
		fclose(f_zhixin);
		fclose(f_result2);
		fclose(f_focus_motion_law);
		fclose(f_q);

		ofstream f_range("range.txt", iostream::out);
		f_range << tj._X;

	}
	catch (ifstream::failure e){
		printf("%s :\n\t", g_file_to_read);
		cout << e.what() << endl;
	}



	if(wait_time != 0)
		prompt_exit(0);
	return 0;
}

