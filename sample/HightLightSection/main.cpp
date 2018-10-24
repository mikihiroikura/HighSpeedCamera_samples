#include <opencv2/opencv.hpp>
#include <fstream>
#include "RS232c.h"
#include <cstring>
#include <math.h>
#include <thread>

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
	vector<double> Heights;
	double params[9];
};

//プロトタイプ宣言
double calc_height(double *params, double X, int Y);
void LightSection(Capture *cap, double params[9], ofstream &fout);

int main() {
	//csvファイルからCalibration結果の読み取り
	double params[9];
	ifstream fin("C:/Users/Mikihiro Ikura/Documents/GitHub/HeightLookupTable/cal_result.csv");
	string line;
	for (int i = 0; i < 3; i++){
		getline(fin, line);
		sscanf(line.c_str(), "%lf,%lf,%lf", &params[0+i*3], &params[1+i*3], &params[2+i*3]);
	}
	///単軸ロボットの設定
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

	///パラメータ入力
	//カメラパラメータ
	int width = 640;
	int height = 480;
	float  fps = 750.0f;
	float gain = 1.0f;
	//キャプチャ用の構造体の宣言
	Capture cap;
	cap.in_img = (cv::Mat(height, width, CV_8UC1, cv::Scalar::all(255)));
	cap.flag = true;
	//高度計算結果保存用ファイル
	ofstream fout("C:/Users/Mikihiro Ikura/Documents/GitHub/HighSpeedCamera/sample/HightLightSection/height_result.csv");
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

	//thread作成
	thread	thr([&] {
		while (cap.flag){
			LightSection(&cap, params, fout);
		}
	});

	///メインループ
	while (cap.flag){
		//printf("%f mm", (cap.height - cap.rob_ini_height));
		cv::imshow("img", cap.in_img);
		int key = cv::waitKey(1);
		if (key == 'q') { cap.flag = false; }
	}

	///メインループ終了
	//thread削除
	if (thr.joinable())thr.join();
	//カメラ停止
	cap.cam.stop();
	cap.cam.disconnect();
	//単軸ロボット切断
	axis.Send("@SRVO0.1\r\n");
	axis.Read_CRLF(buf, 256);
	printf(buf);
	return 0;
}

double calc_height(double *params,double X,int Y) {
	double y = (double)Y;
	double h = (*(params)*pow(y,2)+*(params+1)*y+*(params+2))*pow(X,2)+ 
			   (*(params + 3)*pow(y, 2) + *(params + 4)*y + *(params + 5))*X+
		       (*(params + 6)*pow(y, 2) + *(params + 7)*y + *(params + 8));
	return h;
}

void LightSection(Capture *cap,double params[9],ofstream &fout) {
	//カメラフレーム取得
	cap->cam.captureFrame(cap->in_img.data);
	//光切断法による輝度重心計算⇒高度計算
	cv::threshold(cap->in_img, cap->out_img, 150.0, 255.0, cv::THRESH_BINARY);
	cv::bitwise_and(cap->in_img, cap->out_img, cap->out_img);
	vector<double> cogs,heights;
	double moment, mass, cog,height;
	for (int i = 0; i < cap->out_img.rows; i++) {
		moment = 0.0;
		mass = 0.0;
		for (size_t j = 0; j < cap->out_img.cols; j++) {
			mass += cap->out_img.data[i*cap->out_img.cols + j];
			moment += j * cap->out_img.data[i*cap->out_img.cols + j];
		}
		if (mass <= 0) { 
			cog = 0;
			height = 0;
		}
		else {
			cog = moment / mass;
			height = calc_height(&params[0], cog, i);
		}
		cogs.push_back(cog);
		heights.push_back(height);
	}
	//copy(cogs.begin(), cogs.end(), cap->CoGs.begin());//Capture構造体のCoGsに保存
	//copy(heights.begin(), heights.end(), cap->Heights.begin());//Capture構造体のHeightsに保存
	cogs.clear();
	//heights.clear();
	//出力ファイルに結果を書きこむ
	for (size_t i = 0; i < heights.size(); i++)
	{
		if (isnan(heights[i])) { fout << ","; }
		else { fout << heights[i] << ","; }
	}
	fout << endl;
	heights.clear();
}

//単軸ロボットをランダムで動かす関数
