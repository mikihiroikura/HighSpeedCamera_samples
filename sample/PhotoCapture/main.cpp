//Camera calibration用に写真を撮る関数
//RS232でMBEDにつなぐ

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
	bool max_h_flg = false;
};

//プロトタイプ宣言
void AxisInchUP(RS232c &axis, Capture &cap, int num);
void AxisInchDOWN(RS232c &axis, Capture &cap, int num);
void TakePicture(Capture *cap, bool *flg);
void AxisToPoint(RS232c &axis, Capture &cap, int pointnum);

int main() {
	//カメラパラメータ
	int width = 640;
	int height = 480;
	float  fps = 750.0f;
	float gain = 1.0f;

	//各種パラメータ
	int counter = 0;
	string offdir = "Photos/Laser_off";
	string ondir = "Photos/Laser_on";
	int num_pic = 1;
	vector<cv::Mat> OFF_Pictures;
	vector<cv::Mat> ON_Pictures;
	vector<float> RobotHeights;
	cv::Mat blank = (cv::Mat(height, width, CV_8UC1, cv::Scalar::all(255)));

	//MBEDマイコンへのRS232接続
	RS232c mbed;
	char buf_mbed[256];
	mbed.Connect("COM4", 9600, 8, NOPARITY);
	printf("MBED Connected");

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
	axis.Send("@ORG.1\r\n");
	axis.Read_CRLF(buf, 256);
	printf(buf);
	Sleep(5000);
	axis.Read_CRLF(buf, 256);
	printf(buf);
	while (1) {
		int H_org;
		axis.Send("@?D0.1\r\n");
		axis.Read_CRLF(buf, 256);
		printf(buf);
		sscanf(buf, "D0.1=%d\r\n", &H_org);
		if (H_org < 100) { break; }
	}

	//キャプチャ用の構造体の宣言
	Capture cap;
	cap.in_img = (cv::Mat(height, width, CV_8UC1, cv::Scalar::all(255)));
	cap.flag = true;
	cap.cam.connect(0);
	cap.cam.setParam(paramTypeCamera::paramInt::WIDTH, width);
	cap.cam.setParam(paramTypeCamera::paramInt::HEIGHT, height);
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

	bool flag = true;
	thread thr(TakePicture, &cap, &flag);

	bool on_flag = false;
	int cnt = 0;//InchUpした回数
	int maxup =10;
	int max_pic_pair = 10;//画像群の獲得個数
	int pic_pair = 0;
	char buf_mirror[7];//ミラーへの入力電圧文字列
	float max_mirrorV = 2.8;//ミラーへの最大電圧
	float min_mirrorV = 0.5;//ミラーへの最小電圧
	float mirror_dV = (max_mirrorV - min_mirrorV) / (float)(maxup-1);//一回更新の時の差分電圧
	float mirrorV = min_mirrorV;//ミラーの入力電圧
	axis.Read_CRLF(buf, 256);
	printf(buf);
	mbed.Send("m");
	Sleep(1);
	snprintf(buf_mirror, 7, "%.5f", mirrorV);//floatの電圧値を7桁のchar文字列に変換
	mbed.Send(buf_mirror);
	Sleep(1);
	mbed.Send("ON!");
	Sleep(1);
	while (1) {
		cv::imshow("img", cap.in_img);
		int key = cv::waitKey(10);
		if (key == 'q')break;
		if (cnt > 0) {
			for (size_t i = 0; i < num_pic; i++)
			{
				Sleep(10);
				mbed.Send("ON!");
				Sleep(1000);
				ON_Pictures.push_back(cap.in_img.clone());
				Sleep(3000);
				mbed.Send("OFF");
				Sleep(1000);
				OFF_Pictures.push_back(cap.in_img.clone());
			}
			RobotHeights.push_back(cap.height);
		}
		if (cnt==0){
			mbed.Send("ON!");
			Sleep(1);
			AxisToPoint(axis, cap, 15);
		}
		else if(cnt>0&&cnt<maxup){
			int point = cnt +15;
			AxisToPoint(axis, cap, point);
		}
		cnt++;
		if (cnt > maxup) {
			//画像の保存
			for (size_t i = 0; i < OFF_Pictures.size(); i++)
			{
				cv::imwrite(offdir + to_string(pic_pair+1)+"/picture" + to_string(int(RobotHeights[i])) + "mm.jpg", OFF_Pictures[i]);
				cv::imwrite(ondir  +to_string(pic_pair + 1) + "/picture" + to_string(int(RobotHeights[i])) + "mm.jpg", ON_Pictures[i]);
			}
			OFF_Pictures.clear();
			ON_Pictures.clear();
			RobotHeights.clear();
			//AxisInchDOWN(axis, cap, 45 * 9);
			cnt = 0;
			pic_pair++;
			if (pic_pair == max_pic_pair) {break;}
			mirrorV += mirror_dV;
			mbed.Send("m");
			Sleep(1);
			snprintf(buf_mirror, 7, "%.5f", mirrorV);//floatの電圧値を7桁のchar文字列に変換
			mbed.Send(buf_mirror);
			Sleep(1);
		}
		//if ((cap.height - cap.rob_ini_height) >= 700) { break; }
	}
	flag = false;
	if (thr.joinable())thr.join();

	cap.cam.stop();
	cap.cam.disconnect();
	//単軸ロボット切断
	axis.Send("@SRVO0.1\r\n");
	axis.Read_CRLF(buf, 256);
	//axis.~RS232c();
	printf(buf);
	//MBED通信切断
	//mbed.~RS232c();
	//画像の保存
	/*for (size_t i = 0; i < OFF_Pictures.size(); i++)
	{
		cv::imwrite(offdir + to_string(int(RobotHeights[i])) + "mm.jpg", OFF_Pictures[i]);
		cv::imwrite(ondir + to_string(int(RobotHeights[i])) + "mm.jpg", ON_Pictures[i]);
	}*/
	
	return 0;
}

//thread内の関数
void TakePicture(Capture *cap, bool *flg) {
	while (*flg) {
		cap->cam.captureFrame(cap->in_img.data);
	}
}

//単軸ロボットのINCH+モード
void AxisInchUP(RS232c &axis, Capture &cap, int num) {
	char buf[256];
	int rob_height;//単位は0.01mmが1
	for (int i = 0; i < num; i++) {
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

//単軸ロボットのINCH-モード
void AxisInchDOWN(RS232c &axis, Capture &cap, int num) {
	char buf[256];
	int rob_height;//単位は0.01mmが1
	for (int i = 0; i < num; i++) {
		axis.Send("@INCH-.1\r\n");
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

//単軸ロボットのポイント指定移動
void AxisToPoint(RS232c &axis, Capture &cap,int pointnum) {
	char command[256] = { '\0' };
	char buf[256];
	int rob_height;
	axis.Read_CRLF(buf, 256);
	printf(buf);//RUN
	sprintf(command, "@START%d.1\r\n", pointnum);
	axis.Send(command);
	axis.Read_CRLF(buf, 256);
	printf(buf);//RUN
	Sleep(5000);
	axis.Read_CRLF(buf, 256);
	printf(buf);//END
	Sleep(1000);
	axis.Send("@?D0.1\r\n");
	axis.Read_CRLF(buf, 256);
	printf(buf);//D0.1=~~
	/*axis.Send("@?D0.1\r\n");
	axis.Read_CRLF(buf, 256);
	printf(buf);*/
	sscanf(buf, "D0.1=%d\r\n", &rob_height);
	cap.height = cap.rob_ini_height + (float)rob_height * 0.01;
}