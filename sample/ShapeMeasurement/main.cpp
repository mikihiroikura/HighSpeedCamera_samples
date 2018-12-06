#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <thread>
#include <string>
#include <vector>
#include "RS232c.h"
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <direct.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <Windows.h>

#ifdef _DEBUG
#define CAMERA_EXT "d.lib"
#else
#define CAMERA_EXT ".lib"
#endif

#define STR_EXP(__A) #__A
#define STR(__A) STR_EXP(__A)
#define CV_VER STR(CV_VERSION_MAJOR) STR(CV_VERSION_MINOR) STR(CV_VERSION_REVISION)
#pragma comment(lib, "opencv_world" CV_VER CAMERA_EXT)

#define MAX_

#include <HSC/baslerClass.hpp>
#pragma comment(lib, "BaslerLib" CAMERA_EXT)

#pragma warning(disable:4996)

//DEFINE�Q
//#define SAVE_IMG_         //�摜�Ɠ����Log�Ɏc��
#define MATLAB_GRAPHICS_  //MATLAB���N�����CLog���v���b�g����

using namespace std;

struct Capture
{
	bool flag;//���C�����[�v���s�t���O
	unsigned int pic_cnt;//�����J�E���^�[
	basler cam;//Basler�N���X
	cv::Mat in_img;//���ؒf�@���͉摜
	cv::Mat out_img;//���ؒf�摜�����o�͉摜
	float rob_ini_height = 30.0f;//mm
	float height_interval = 10.0f;//mm
	float height = rob_ini_height;//mm
	vector<double> CoGs;//�s���Ƃ̋P�x�d�S���W
	vector<vector<double>> IdealPixs;//�c�ݕ␳�s�N�Z�����W
	vector<double> lambdas;//�����̎��̔{��
	vector<double> Heights_cam;//���x@�J�������W�n
	vector<double> Heights_world;//���x@World���W�n
	vector<cv::Mat> Worlds;//World���W�n
	vector<int> row_num;
	bool max_h_flg = false;
	//���჌���Y�����p�����[�^�[
	vector<double> map_coefficient;
	vector<double> stretch_mat;
	vector<double> distortion;
	vector<double> laser_plane;
	vector<cv::Mat> Rots;
	vector<double> Rot_height;
	//Log�i�[�p�z��
	vector<double> Log_times;
	vector<vector<int>> Row_num_Logs;
	vector<vector<double>> Height_Logs;
	vector<vector<cv::Mat>> Worlds_Logs;
	vector<cv::Mat> Pictures;
	vector<bool> Processed;
};
//�O���[�o���ϐ�
double result, logtime ,det;
//���ؒf�@�͈͎w��
int sta_row = 80;
int end_row = 400;
int cnt_row = (end_row - sta_row) / 2;


//�v���g�^�C�v�錾
void TakePicture(Capture *cap, bool *flg);
void CalcHeights(Capture *cap);
void OutPutLogs(Capture *cap, bool *flg);

int main() {
	LARGE_INTEGER freq,start,end;
	if (!QueryPerformanceFrequency(&freq)) { return 0; }// �P�ʏK��
	//�J�����p�����[�^�[
	int width = 640;
	int height = 480;
	float  fps = 750.0f;
	float gain = 1.0f;

	//�L���v�`���p�̍\���̂̐錾
	Capture cap;
	cap.in_img = (cv::Mat(height, width, CV_8UC1, cv::Scalar::all(255)));
	cap.flag = true;
	cap.pic_cnt = 0;
	cap.Processed.push_back(false);//���False��ǉ����Ă���
	cap.cam.connect(0);
	cap.cam.setParam(paramTypeCamera::paramInt::WIDTH, width);
	cap.cam.setParam(paramTypeCamera::paramInt::HEIGHT, height);
	cap.CoGs.resize(height, 0);
	cap.cam.setParam(paramTypeBasler::FastMode::SensorReadoutModeFast);
	cap.cam.setParam(paramTypeBasler::Param::ExposureTime, 1280.0f);
	cap.cam.setParam(paramTypeCamera::paramFloat::FPS, fps);
	cap.cam.setParam(paramTypeCamera::paramFloat::GAIN, gain);
	cap.cam.setParam(paramTypeBasler::AcquisitionMode::EnableAcquisitionFrameRate);//�t���[�����[�g���Œ肵�ăp�����[�^���߂�
	cap.cam.setParam(paramTypeBasler::GrabStrategy::LatestOnlyFrame);//OnebyOne:�B���摜���̂ĂȂ��Œx��Ă��o�b�t�@�ɒ��߂�1�������� LatestOnlyFrame:��Ƀo�b�t�@���X�V���čŐV�̉摜���B�葱����
																	 /*cap.cam.setParam(paramTypeBasler::CaptureType::BayerBGGrab);
																	 cap.cam.setParam(paramTypeBasler::CaptureType::ColorBGRGrab);*/
	cap.cam.setParam(paramTypeBasler::CaptureType::MonocroGrab);
	cap.cam.parameter_all_print();
	//Calibration���ʂ̊i�[
	double a = 0, b = 0, c = 0, d = 0,h = 0;
	double R[9];
	FILE* fp;
	fp = fopen("cameraparams.csv", "r");
	fscanf(fp, "%lf,%lf,%lf,%lf,", &a, &b, &c, &d);
	cap.map_coefficient.push_back(a);
	cap.map_coefficient.push_back(b);
	cap.map_coefficient.push_back(c);
	cap.map_coefficient.push_back(d);
	fscanf(fp, "%lf,%lf,%lf,%lf,", &a, &b, &c, &d);//�s��̏o�͂���鏇�Ԃɒ���
	cap.stretch_mat.push_back(a);
	cap.stretch_mat.push_back(c);
	cap.stretch_mat.push_back(b);
	cap.stretch_mat.push_back(d);
	fscanf(fp, "%lf,%lf,", &a, &b);
	cap.distortion.push_back(a);
	cap.distortion.push_back(b);
	for (size_t i = 0; i < 11; i++)//�����͎ʐ^�̖�������Rot�����o��
	{
		fscanf(fp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,",&h, &R[0], &R[1], &R[2], &R[3], &R[4], &R[5], &R[6], &R[7], &R[8]);
		cv::Mat Rs(cv::Size(3, 3), CV_64F, R);
		//Rs = Rs.inv();//MATLAB����̏o�͎��ɓ]�u���Ƃ����`�ɂȂ�̂ł����͂���Ȃ��Ȃ�
		cap.Rots.push_back(Rs.clone());
		cap.Rot_height.push_back(h);
	}
	
	fscanf(fp, "%lf,%lf,%lf,", &a, &b, &c);
	cap.laser_plane.push_back(a);
	cap.laser_plane.push_back(b);
	cap.laser_plane.push_back(c);
	fclose(fp);

	//det�̌v�Z������
	det = 1.0 / (cap.stretch_mat[0] - cap.stretch_mat[1] * cap.stretch_mat[2]);//�O�ł��

	//�J�����N��
	cap.cam.start();

	//���ʕۑ��p�t�@�C��������t���ō쐬���J��
	FILE* fr;
	time_t timer;
	struct tm now; 
	struct tm *local;
	timer = time(NULL);
	localtime_s(&now, &timer);
	char dir0[256];
	strftime(dir0, 256, "C:/Users/Mikihiro Ikura/Documents/GitHub/HighSpeedCamera/sample/ShapeMeasurement/results/%y%m%d", &now);
	_mkdir(dir0);
	char dir[256];
	strftime(dir, 256, "C:/Users/Mikihiro Ikura/Documents/GitHub/HighSpeedCamera/sample/ShapeMeasurement/results/%y%m%d/%H%M%S", &now);
	_mkdir(dir);
	char dir2[256];
	strftime(dir2, 256, "C:/Users/Mikihiro Ikura/Documents/GitHub/HighSpeedCamera/sample/ShapeMeasurement/results/%y%m%d/%H%M%S/Pictures", &now);
	_mkdir(dir2);
	char filename[256];
	strftime(filename, 256, "results/%y%m%d/%H%M%S/LS_result.csv", &now);
	fr = fopen(filename, "w");
	//
	bool flag = true;
	thread thr(TakePicture, &cap, &flag);
	thread thr2(OutPutLogs, &cap, &flag);
	Sleep(1);//thread�̂�1ms���s���C�摜���i�[������
	if (!QueryPerformanceCounter(&start)) { return 0; }
	//���C�����[�v
	while (flag)
	{
		if (cap.Processed[cap.pic_cnt]){//�ʐ^������Ƃ���
			CalcHeights(&cap);//���x�v�Z
			if (!QueryPerformanceCounter(&end)) { return 0; }
			logtime = (double)(end.QuadPart - start.QuadPart) / freq.QuadPart;
			cap.Log_times.push_back(logtime);
		}
	}
	if (thr.joinable())thr.join();
	if (thr2.joinable())thr2.join();

	cap.cam.stop();
	cap.cam.disconnect();

	//cap����Log��ۑ�
	printf("saving Logs...   ");
	for (int i = 0; i < cap.Log_times.size(); i++){
		fprintf(fr, "%lf,\n", cap.Log_times[i]);
		for (int j = 0; j < cap.Row_num_Logs[i].size(); j++){
			fprintf(fr, "%d,", cap.Row_num_Logs[i][j]);
		}
		fprintf(fr, "\n");
		for (int j = 0; j < cap.Worlds_Logs[i].size(); j++){
			fprintf(fr, "%lf,", cap.Worlds_Logs[i][j].at<double>(0,0));
		}
		fprintf(fr, "\n");
		for (int j = 0; j < cap.Worlds_Logs[i].size(); j++) {
			fprintf(fr, "%lf,", cap.Worlds_Logs[i][j].at<double>(0, 1));
		}
		fprintf(fr, "\n");
		for (int j = 0; j < cap.Worlds_Logs[i].size(); j++) {
			fprintf(fr, "%lf,", cap.Worlds_Logs[i][j].at<double>(0, 2));
		}
		fprintf(fr, "\n");
	}
	printf("Logs finish!\n");
	fclose(fr);

	//�摜�𓮉�ۑ�
	#ifdef SAVE_IMG_
	printf("saving Imgs...   ");
	char videoname[256];
	char picname[256];
	char picsubname[128];
	strftime(videoname, 256, "results/%y%m%d/%H%M%S/video.avi", &now);
	strftime(picsubname, 128, "results/%y%m%d/%H%M%S/Pictures/frame", &now);
	cv::VideoWriter writer(videoname, cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 750.0, cv::Size(640, 480), false);//���m�N���Ȃ̂�False�w��
	if (!writer.isOpened()) { return -1; }
	for (int i = 0; i < cap.Pictures.size(); i++) {
		writer << cap.Pictures[i].clone();
		sprintf(picname, "%s%d.jpg", picsubname, i);
		cv::imwrite(picname, cap.Pictures[i]);
	}
	printf("Imgs finish!\n");
	writer.release();
	#endif // SAVE_IMG_

	//MATLAB��Log�̏o�͂�����
	#ifdef MATLAB_GRAPHICS_
	printf("MATLAB Open...   ");
	int ret = 0;
	char matdir[256] = "C:\\Users\\Mikihiro Ikura\\Documents\\GitHub\\ShowLSLogs\\LS_result.csv";
	int cpy = CopyFileA(filename, matdir, FALSE);
	if (cpy) { printf("Copy success!   "); }
	else { printf("Copy failed...   "); }
	char matlabcmd[256] = "matlab -sd \"C:\\Users\\Mikihiro Ikura\\Desktop\" -r plot_sin -nosplash";
	ret = system(matlabcmd);
	if (ret == 0) { printf("MATLAB activate! \n"); }
	else{printf("error! \n");}
	#endif // MATLAB_GRAPHICS_

	printf("ALL FINALIZE ! \n");
	system("pause");
	return 0;
}

//thread���̊֐�
//�����Ccap�Ɏʐ^���i�[����
void TakePicture(Capture *cap, bool *flg) {
	while (*flg) {
		cap->Pictures.push_back((cv::Mat(480, 640, CV_8UC1, cv::Scalar::all(255))));
		cap->cam.captureFrame(cap->Pictures[cap->Pictures.size()-1].data);
		cap->Processed[cap->Pictures.size() - 1] = true;//���ǉ����ꂽ�ʐ^�̔ԍ�������True�ɂ���
		cap->Processed.push_back(false);//�ŐV�ԍ��̎��̔ԍ�������False�Ƃ��Ă���
	}
}

//thread2���̊֐�
//�W���o�͂Ō��݂̎ʐ^�C�����ƍ��x
void OutPutLogs(Capture *cap ,bool *flg) {
	while (1)
	{
		cv::imshow("img", cap->in_img);
		int key = cv::waitKey(1);
		if (key == 'q') {
			*flg = false;
			break;
		}
		if (cap->Height_Logs.size()>0){
			if (cap->Height_Logs[cap->Height_Logs.size() - 1].size() == 0) { printf("Time: %lf s  Height: NONE\n", &logtime); }
			else {
				result = cap->Worlds_Logs[cap->Worlds_Logs.size() - 1][0].at<double>(0, 2);
				printf("Time: %lf s  Height: %lf mm\n", logtime, result);
			}
		}
		
	}
}

//main loop���ł̍��x�v�Z
void CalcHeights(Capture *cap) {
	//�O��g�p�����z���Clear
	cap->CoGs.clear();
	cap->Heights_cam.clear();
	cap->Heights_world.clear();
	cap->IdealPixs.clear();
	cap->Worlds.clear();
	cap->row_num.clear();
	//�摜�z��Q���玟�̏����J�E���^�[�̔ԍ��̉摜��in_img�Ɋi�[����
	cap->in_img = cap->Pictures[cap->pic_cnt];	
	//�P�x�d�S�̌v�Z
	cv::threshold(cap->in_img, cap->out_img, 150.0, 255.0, cv::THRESH_BINARY);
	cv::bitwise_and(cap->in_img, cap->out_img, cap->out_img);
	double moment, mass, cog;
	for (int i = sta_row; i < end_row; i++) {
		moment = 0.0;
		mass = 0.0;
		for (int j = 0; j < cap->out_img.cols; j++) {
			mass += cap->out_img.data[i*cap->out_img.cols + j];
			moment += j * cap->out_img.data[i*cap->out_img.cols + j];
		}
		if (mass <= 0) { cog = 0; }
		else {
			cog = moment / mass;
			cap->row_num.push_back(i);
		}
		cap->CoGs.push_back(cog);

	}
	//�P�x�d�S�˗��z�s�N�Z�����W�ɕϊ�
	vector<double> idpixs;
	double u, v, w;//����for���ł��g��
	double ud, vd;
	for (int i = 0; i < cap->CoGs.size(); i++)
	{
		if (cap->CoGs[i] == 0) { continue; }
		ud = cap->CoGs[i] - cap->distortion[0];
		vd = (double)(i + sta_row) - cap->distortion[1];
		u = det * (ud - cap->stretch_mat[1] * vd);
		v = det * (-cap->stretch_mat[2] * ud + cap->stretch_mat[0] * vd);
		idpixs.push_back(u);
		idpixs.push_back(v);
		cap->IdealPixs.push_back(idpixs);
		idpixs.clear();
	}
	//���z�s�N�Z�����W�˒����̎��ƃ��[�U�[���ʂ̋���
	double phi, lambda, h;
	for (size_t i = 0; i < cap->IdealPixs.size(); i++)
	{
		u = cap->IdealPixs[i][0];
		v = cap->IdealPixs[i][1];
		phi = pow((pow(u, 2) + pow(v, 2)), 0.5);
		w = cap->map_coefficient[0] + cap->map_coefficient[1] * pow(phi, 2)
			+ cap->map_coefficient[2] * pow(phi, 3) + cap->map_coefficient[3] * pow(phi, 4);
		lambda = -cap->laser_plane[2] / (cap->laser_plane[0] * u + cap->laser_plane[1] * v - w);
		cap->lambdas.push_back(lambda);
		h = lambda * w;
		cap->Heights_cam.push_back(h);
	}
	//Camera��World�ւ̉�]�s��̑I��
	//double min_h=1000;
	//int min_idx=0;
	/*for (int j = 0; j < cap->Rot_height.size(); j++)
	{
	if (cap->Heights.size() == 0) { continue; }
	if (min_h>abs(cap->Heights[0]-cap->Rot_height[j]))
	{
	min_h = abs(cap->Heights[0] - cap->Rot_height[j]);
	min_idx = j;
	}
	}*/
	//�J�������W�n���S��World���W�ɕϊ�
	cv::Mat campnt, wldpnt;
	double hw;
	for (size_t i = 0; i < cap->Heights_cam.size(); i++)
	{
		campnt = (cv::Mat_<double>(1, 3) << cap->lambdas[i] * cap->IdealPixs[i][0], cap->lambdas[i] * cap->IdealPixs[i][1], cap->Heights_cam[i]);
		wldpnt = campnt * cap->Rots[0];
		hw = wldpnt.clone().at<double>(0, 2);
		cap->Worlds.push_back(wldpnt.clone());
		cap->Heights_world.push_back(hw);
	}
	cap->Height_Logs.push_back(cap->Heights_world);
	cap->Worlds_Logs.push_back(cap->Worlds);
	cap->Row_num_Logs.push_back(cap->row_num);
	//�����J�E���^�[�Ə������ʂ̍X�V
	cap->pic_cnt++;	
}
