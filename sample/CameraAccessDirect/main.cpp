
#include <opencv2/opencv.hpp>
#include <thread>
#include <vector>
#include <fstream>
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
	bool max_h_flg=false;
};

//プロトタイプ宣言
void capthread(Capture *cap);
void HeightCalibration(Capture *cap, ofstream &fout,RS232c &axis);
void AxisInchUP(RS232c &axis, Capture &cap, int num);

//グローバル変数


int main() {
	//カメラパラメーター
	int width = 640;
	int height = 480;
	float  fps = 750.0f;
	float gain = 1.0f;
	//Calibration用ファイル
	ofstream fout("light_section_result.csv");
	//単軸ロボットのRS232接続
	RS232c axis;
	char buf[256];
	axis.Connect("COM3", 38400, ODDPARITY, 0, 0, 0, 20000, 20000);
	//単軸ロボットサーボ入力
	axis.Send("@SRVO1.1\r\n");
	axis.Read_CRLF(buf, 256);
	printf(buf);
	Sleep(3000);
	//単軸ロボット原点回帰
	/*axis.Send("@ORG.1\r\n");
	axis.Read_CRLF(buf, 256);
	printf(buf);
	axis.Read_CRLF(buf, 256);
	printf(buf);*/
	//Sleep(5000);

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
	//カメラ起動
	cap.cam.start();
	
	//ここに単軸ロボットの高度を変更するThreadを作る//
	/*thread thr2(ChangeRobHeight,&cap);
	thread thr(capthread,&cap);*/
	//thread thr(HeightCalibration, axis, &cap, 50);
	

	while (1) {
		printf("%f mm",(cap.height - cap.rob_ini_height));
		cap.cam.captureFrame(cap.in_img.data);
		cv::imshow("img", cap.in_img);
		int key = cv::waitKey(2);
		if (key == 'q'||(cap.height-cap.rob_ini_height)>=700)break;
		Sleep(3000);
		HeightCalibration(&cap, fout, axis);
		if (cap.max_h_flg) { break; }
	}
	cap.flag = false;
	/*if (thr.joinable())thr.join();
	if (thr2.joinable())thr2.join();*/

	cap.cam.stop();
	cap.cam.disconnect();

	//単軸ロボット切断
	axis.Send("@SRVO0.1\r\n");
	axis.Read_CRLF(buf, 256);
	//axis.~RS232c();
	printf(buf);
	return 0;
}

void capthread(Capture *cap) {
	while (cap->flag) {
		/*double threshold = 128.0;
		double max_val = 255.0;*/
		cap->cam.captureFrame(cap->in_img.data);
		//光切断法画像処理
		cv::threshold(cap->in_img, cap->out_img, 128.0, 255.0, cv::THRESH_BINARY);
		cv::bitwise_and(cap->in_img, cap->out_img, cap->out_img);
		vector<double> cogs;
		double moment,mass,cog;
		for (size_t i = 0; i < cap->out_img.rows; i++){
			moment = 0.0;
			mass = 0.0;
			for (size_t j = 0; j < cap->out_img.cols; j++){
				mass += cap->out_img.data[i*cap->out_img.cols + j];
				moment += j*cap->out_img.data[i*cap->out_img.cols + j];
			}
			cog = moment / mass;
			cogs.push_back(cog);
		}
		memcpy(&cap->CoGs, &cogs, sizeof(cap->CoGs));
		cogs.clear();
	}
}


void HeightCalibration(Capture *cap, ofstream &fout, RS232c &axis) {
	//カメラからフレーム取得
	cap->cam.captureFrame(cap->in_img.data);
	//光切断法による行ごとの輝度重心の計算
	cv::threshold(cap->in_img, cap->out_img, 128.0, 255.0, cv::THRESH_BINARY);
	cv::bitwise_and(cap->in_img, cap->out_img, cap->out_img);
	vector<double> cogs;
	double moment, mass, cog;
	for (size_t i = 0; i < cap->out_img.rows; i++) {
		moment = 0.0;
		mass = 0.0;
		for (size_t j = 0; j < cap->out_img.cols; j++) {
			mass += cap->out_img.data[i*cap->out_img.cols + j];
			moment += j * cap->out_img.data[i*cap->out_img.cols + j];
		}
		cog = moment / mass;
		cogs.push_back(cog);
	}
	/*
	for (int i = 0; i < cogs.size(); i++) {
		cap->CoGs.push_back(cogs[i]);
	}
	*/
	memcpy(&(cap->CoGs[0]), &cogs[0], sizeof(cap->CoGs));//Capture構造体のCoGsに保存
	cogs.clear();
	//出力ファイルに結果を保存する
	fout << cap->height << ",";
	for (size_t i = 0; i < cap->CoGs.size(); i++){
		fout << cap->CoGs[i] << ",";
	}
	fout << endl;
	//cap->CoGs.clear();
	//ここに単軸ロボットを制御する関数を書く//
	AxisInchUP(axis, *cap, 50);
}

//単軸ロボットのINCH+モード
void AxisInchUP(RS232c &axis,Capture &cap,int num) {
	char buf[256];
	int rob_height;//単位は0.01mmが1
	for (int i = 0; i < num; i++){
		axis.Send("@INCH+.1\r\n");
		axis.Read_CRLF(buf, 256);
		printf(buf);
		axis.Read_CRLF(buf, 256);
		printf(buf);
		if (buf[0] == 'N'&&buf[1] == 'G') {
			cap.max_h_flg = true;
			break;
		}
		/*axis.Send("@?D0.1\r\n");
		axis.Read_CRLF(buf, 256);
		sscanf(buf, "D0.1=%d\r\n", &rob_height);*/
		
	}
	axis.Send("@?D0.1\r\n");
	axis.Read_CRLF(buf, 256);
	printf(buf);
	sscanf(buf, "D0.1=%d\r\n", &rob_height);
	cap.height = cap.rob_ini_height + (float)rob_height * 0.01;

}