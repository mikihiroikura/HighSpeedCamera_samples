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
void TakePicture(Capture *cap, bool flg);

int main() {
	//カメラパラメータ
	int width = 640;
	int height = 480;
	float  fps = 750.0f;
	float gain = 1.0f;

	//各種パラメータ
	int counter = 0;
	string offdir = "Photos/Laser_off/picure";
	string ondir = "Photos/Laser_on/picture";
	int num_pic = 1;
	vector<cv::Mat> OFF_Pictures;
	vector<cv::Mat> ON_Pictures;
	vector<float> RobotHeights;
	cv::Mat blank = (cv::Mat(height, width, CV_8UC1, cv::Scalar::all(255)));;

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
	axis.Read_CRLF(buf, 256);
	printf(buf);
	

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

	/*bool flag = true;
	thread thr(TakePicture, &cap, flag);*/

	bool on_flag = false;

	while (1) {
		cap.cam.captureFrame(cap.in_img.data);
		cv::imshow("img", cap.in_img);
		int key = cv::waitKey(1);
		if (key == 'q')break;
		for (size_t i = 0; i < num_pic; i++)
		{
			if (on_flag) {
				ON_Pictures.push_back(cap.in_img.clone());
			}
			else {
				OFF_Pictures.push_back(cap.in_img.clone());
			}
		}
		RobotHeights.push_back(cap.height);
		AxisInchUP(axis, cap, 45);
		if ((cap.height - cap.rob_ini_height) >= 700){
			if (on_flag)break;
			else{
				//単軸ロボット原点回帰
				axis.Send("@ORG.1\r\n");
				axis.Read_CRLF(buf, 256);
				printf(buf);
				axis.Read_CRLF(buf, 256);
				printf(buf);
				on_flag = true;
				cap.height = cap.rob_ini_height;
			}
		}
	}
	//flag = false;
	//if (thr.joinable())thr.join();

	cap.cam.stop();
	cap.cam.disconnect();
	//単軸ロボット切断
	axis.Send("@SRVO0.1\r\n");
	axis.Read_CRLF(buf, 256);
	//axis.~RS232c();
	printf(buf);
	//画像の保存
	for (size_t i = 0; i < OFF_Pictures.size(); i++)
	{
		cv::imwrite(offdir + to_string(int(RobotHeights[i])) + "mm.jpg", OFF_Pictures[i]);
		cv::imwrite(ondir + to_string(int(RobotHeights[i+OFF_Pictures.size()])) + "mm.jpg", ON_Pictures[i]);
	}
	
	return 0;
}

//thread内の関数
void TakePicture(Capture *cap, bool flg) {
	while (flg) {
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