#include <opencv2/opencv.hpp>
#include <thread>
#include <string>
#include <vector>
#include "RS232c.h"
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <direct.h>

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
	vector<double> Heights_cam;
	vector<double> Heights_world;
	vector<cv::Mat> Worlds;
	vector<int> row_num;
	bool max_h_flg = false;
	vector<double> map_coefficient;
	vector<double> stretch_mat;
	vector<double> distortion;
	vector<double> laser_plane;
	vector<cv::Mat> Rots;
	vector<double> Rot_height;
	vector<double> Log_times;
	vector<vector<int>> Row_num_Logs;
	vector<vector<double>> Height_Logs;
};
//グローバル変数
double result, logtime ,det;


//プロトタイプ宣言
void TakePicture(Capture *cap, bool *flg);
void CalcHeights(Capture *cap);
void OutPutLogs(Capture *cap, bool *flg);

int main() {
	LARGE_INTEGER freq,start,end;
	if (!QueryPerformanceFrequency(&freq)) { return 0; }// 単位習得
	//カメラパラメーター
	int width = 640;
	int height = 480;
	float  fps = 750.0f;
	float gain = 1.0f;

	//キャプチャ用の構造体の宣言
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
	cap.cam.setParam(paramTypeBasler::AcquisitionMode::EnableAcquisitionFrameRate);//フレームレートを固定してパラメータ決める
	cap.cam.setParam(paramTypeBasler::GrabStrategy::LatestOnlyFrame);//OnebyOne:撮像画像を捨てないで遅れてもバッファに貯めた1枚ずつ送る LatestOnlyFrame:常にバッファを更新して最新の画像を撮り続ける
																	 /*cap.cam.setParam(paramTypeBasler::CaptureType::BayerBGGrab);
																	 cap.cam.setParam(paramTypeBasler::CaptureType::ColorBGRGrab);*/
	cap.cam.setParam(paramTypeBasler::CaptureType::MonocroGrab);
	cap.cam.parameter_all_print();
	//Calibration結果の格納
	double a = 0, b = 0, c = 0, d = 0,h = 0;
	double R[9];
	FILE* fp;
	fp = fopen("cameraparams.csv", "r");
	fscanf(fp, "%lf,%lf,%lf,%lf,", &a, &b, &c, &d);
	cap.map_coefficient.push_back(a);
	cap.map_coefficient.push_back(b);
	cap.map_coefficient.push_back(c);
	cap.map_coefficient.push_back(d);
	fscanf(fp, "%lf,%lf,%lf,%lf,", &a, &b, &c, &d);//行列の出力される順番に注意
	cap.stretch_mat.push_back(a);
	cap.stretch_mat.push_back(c);
	cap.stretch_mat.push_back(b);
	cap.stretch_mat.push_back(d);
	fscanf(fp, "%lf,%lf,", &a, &b);
	cap.distortion.push_back(a);
	cap.distortion.push_back(b);
	for (size_t i = 0; i < 11; i++)//ここは写真の枚数分のRotを取り出す
	{
		fscanf(fp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,",&h, &R[0], &R[1], &R[2], &R[3], &R[4], &R[5], &R[6], &R[7], &R[8]);
		cv::Mat Rs(cv::Size(3, 3), CV_64F, R);
		//Rs = Rs.inv();//MATLABからの出力時に転置をとった形になるのでここはいらなくなる
		cap.Rots.push_back(Rs.clone());
		cap.Rot_height.push_back(h);
	}
	
	fscanf(fp, "%lf,%lf,%lf,", &a, &b, &c);
	cap.laser_plane.push_back(a);
	cap.laser_plane.push_back(b);
	cap.laser_plane.push_back(c);
	fclose(fp);

	//detの計算をする
	det = 1.0 / (cap.stretch_mat[0] - cap.stretch_mat[1] * cap.stretch_mat[2]);//外でやる

	//カメラ起動
	cap.cam.start();

	//結果保存用ファイルを日時付きで作成し開く
	FILE* fr;
	time_t timer;
	struct tm *local;
	timer = time(NULL);
	local = localtime(&timer);
	char dir[256];
	sprintf(dir, "C:/Users/Mikihiro Ikura/Documents/GitHub/HighSpeedCamera/sample/ShapeMeasurement/results/%4d%2d%2d", local->tm_year + 1900, local->tm_mon + 1, local->tm_mday);
	_mkdir(dir);
	char filename[256];
	sprintf(filename, "results/%4d%2d%2d/%2d%2d_LS_result.csv", local->tm_year + 1900, local->tm_mon + 1, local->tm_mday, local->tm_hour, local->tm_min);
	fr = fopen(filename, "w");
	//
	bool flag = true;
	thread thr(TakePicture, &cap, &flag);
	thread thr2(OutPutLogs, &cap, &flag);

	if (!QueryPerformanceCounter(&start)) { return 0; }
	//メインループ
	while (flag)
	{
		CalcHeights(&cap);//高度計算
		if (!QueryPerformanceCounter(&end)) { return 0; }
		logtime = (double)(end.QuadPart - start.QuadPart) / freq.QuadPart;
		cap.Log_times.push_back(logtime);
	}
	if (thr.joinable())thr.join();
	if (thr2.joinable())thr2.join();

	cap.cam.stop();
	cap.cam.disconnect();

	//cap内のLogを保存
	printf("saving Logs...\n");
	for (size_t i = 0; i < cap.Height_Logs.size(); i++){
		fprintf(fr, "%lf,", cap.Log_times[i]);
		for (size_t j = 0; j < cap.Height_Logs[i].size(); j++){
			fprintf(fr, "%d,", cap.Row_num_Logs[i][j]);
		}
		fprintf(fr, "\n");
		for (size_t j = 0; j < cap.Height_Logs[i].size(); j++){
			fprintf(fr, "%lf,", cap.Height_Logs[i][j]);
		}
		fprintf(fr, "\n");
	}
	printf("finish!\n");

	return 0;
}

//thread内の関数
void TakePicture(Capture *cap, bool *flg) {
	while (*flg) {
		cap->cam.captureFrame(cap->in_img.data);
	}
}

//thread2内の関数
void OutPutLogs(Capture *cap ,bool *flg) {
	while (1)
	{
		cv::imshow("img", cap->in_img);
		int key = cv::waitKey(1);
		if (key == 'q') {
			*flg = false;
			break;
		}
		if (cap->Height_Logs[cap->Height_Logs.size() - 1].size() == 0) { printf("Time: %lf s  Height: NONE\n", &logtime); }
		else {
			result = cap->Height_Logs[cap->Height_Logs.size() - 1][0];
			printf("Time: %lf s  Height: %lf mm\n", logtime, result);
		}
	}
}

//main loop内でのh高度計算
void CalcHeights(Capture *cap) {
	cap->CoGs.clear();
	cap->Heights_cam.clear();
	cap->Heights_world.clear();
	cap->IdealPixs.clear();
	cap->Worlds.clear();
	cap->row_num.clear();
	//輝度重心計算する行の範囲指定
	int sta_row = 200;
	int end_row = 280;
	//輝度重心の計算
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
	//輝度重心⇒理想ピクセル座標に変換
	vector<double> idpixs;
	double u, v,w;//次のfor内でも使う
	double ud,vd;
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
	//理想ピクセル座標⇒直線の式とレーザー平面の求解
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
		cap->Heights_cam.push_back(h);
	}
	//Camera⇒Worldへの回転行列の選択
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
	//カメラ座標系中心のWorld座標に変換
	cv::Mat campnt,wldpnt;
	double hw;
	for (size_t i = 0; i < cap->Heights_cam.size(); i++)
	{
		campnt = (cv::Mat_<double>(1, 3) << cap->lambdas[i]*cap->IdealPixs[i][0], cap->lambdas[i] * cap->IdealPixs[i][1], cap->Heights_cam[i]);
		wldpnt = campnt * cap->Rots[0];
		hw = wldpnt.clone().at<double>(0,2);
		cap->Worlds.push_back(wldpnt.clone());
		cap->Heights_world.push_back(hw);
	}
	cap->Height_Logs.push_back(cap->Heights_world);
	cap->Row_num_Logs.push_back(cap->row_num);
}