#include <opencv2/opencv.hpp>
#include <thread>
#include <string>
#include <vector>
#include "RS232c.h"

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

using namespace std;

struct Capture
{
	bool flag;
	basler cam;
	cv::Mat in_img;
	cv::Mat out_img;
	float rob_ini_height = 30.0f;//mm
	float height_interval = 10.0f;//mm
	float height = rob_ini_height;//mm
	vector<double> CoGs;
	vector<vector<double>> IdealPixs;
	vector<double> lambdas;
	vector<double> Heights;
	vector<cv::Mat> Worlds;
	bool max_h_flg = false;
	vector<double> map_coefficient;
	vector<double> stretch_mat;
	vector<double> distortion;
	vector<double> laser_plane;
	cv::Mat Rot;
};

//�v���g�^�C�v�錾
void TakePicture(Capture *cap, bool *flg);
void CalcHeights(Capture *cap);

int main() {
	//�e��p�����[�^
	double result,time;
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
	double a = 0, b = 0, c = 0, d = 0;
	double R[9];
	FILE* fp;
	fp = fopen("cameraparams.csv", "r");
	fscanf(fp, "%lf,%lf,%lf,%lf,", &a, &b, &c, &d);
	cap.map_coefficient.push_back(a);
	cap.map_coefficient.push_back(b);
	cap.map_coefficient.push_back(c);
	cap.map_coefficient.push_back(d);
	fscanf(fp, "%lf,%lf,%lf,%lf,", &a, &b, &c, &d);
	cap.stretch_mat.push_back(a);
	cap.stretch_mat.push_back(b);
	cap.stretch_mat.push_back(c);
	cap.stretch_mat.push_back(d);
	fscanf(fp, "%lf,%lf,", &a, &b);
	cap.distortion.push_back(a);
	cap.distortion.push_back(b);
	fscanf(fp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,", &R[0], &R[1], &R[2], &R[3], &R[4], &R[5], &R[6], &R[7], &R[8]);
	cv::Mat Rs(cv::Size(3, 3),CV_64F, R);
	Rs = Rs.inv();
	cap.Rot = Rs.clone();
	fscanf(fp, "%lf,%lf,%lf,", &a, &b, &c);
	cap.laser_plane.push_back(a);
	cap.laser_plane.push_back(b);
	cap.laser_plane.push_back(c);
	fclose(fp);

	//�J�����N��
	cap.cam.start();

	//���ʕۑ��p�t�@�C���J��
	FILE* fr;
	fr = fopen("result.csv", "w");

	bool flag = true;
	thread thr(TakePicture, &cap, &flag);

	if (!QueryPerformanceCounter(&start)) { return 0; }
	while (1)
	{
		cv::imshow("img", cap.in_img);
		int key = cv::waitKey(1);
		if (key == 'q')break;
		CalcHeights(&cap);
		if (!QueryPerformanceCounter(&end)) { return 0; }
		time = (double)(end.QuadPart - start.QuadPart) / freq.QuadPart;
		if (cap.Worlds.size()==0){printf("Time: %lf s  Height: NONE\n",time);}
		else{
			result = cap.Worlds[0].at<double>(0, 2);
			printf("Time: %lf s  Height: %lf mm\n",time,result);
			fprintf(fr,"%lf,%lf\n", time, result);
		}
	}
	flag = false;
	if (thr.joinable())thr.join();

	cap.cam.stop();
	cap.cam.disconnect();

	return 0;
}

//thread���̊֐�
void TakePicture(Capture *cap, bool *flg) {
	while (*flg) {
		cap->cam.captureFrame(cap->in_img.data);
	}
}

void CalcHeights(Capture *cap) {
	cap->CoGs.clear();
	cap->Heights.clear();
	cap->IdealPixs.clear();
	cap->Worlds.clear();
	//�P�x�d�S�v�Z����s�͈͎̔w��
	int sta_row = 200;
	int end_row = 280;
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
		else { cog = moment / mass; }
		cap->CoGs.push_back(cog);
	}
	//�P�x�d�S�˗��z�s�N�Z�����W�ɕϊ�
	vector<double> idpixs;
	double u, v,w;//����for���ł��g��
	double det,ud,vd;
	for (int i = 0; i < cap->CoGs.size(); i++)
	{
		if (cap->CoGs[i] == 0) { continue; }
		det = 1.0 / (cap->stretch_mat[0] - cap->stretch_mat[1] * cap->stretch_mat[2]);
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
	double phi,lambda,h;
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
		cap->Heights.push_back(h);
	}
	//�J�������W�n���S��World���W�ɕϊ�
	cv::Mat campnt,wldpnt;
	for (size_t i = 0; i < cap->Heights.size(); i++)
	{
		campnt = (cv::Mat_<double>(1, 3) << cap->lambdas[i]*cap->IdealPixs[i][0], cap->lambdas[i] * cap->IdealPixs[i][1], cap->Heights[i]);
		wldpnt = campnt * cap->Rot;
		cap->Worlds.push_back(wldpnt.clone());
	}
}