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
#define _USE_MATH_DEFINES
#include <math.h>
#define GLFW_NO_GLU

#pragma comment(lib, "glew32.lib")
#include <GL/glew.h>
#include "GLFW/glfw3.h"
#include <glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <gtc/matrix_transform.hpp> 
#include <gtx/transform.hpp>


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


//DEFINE群
#define ROBOT_MOVE_//ロボットを動かす
#define SAVE_IMG_         //画像と動画をLogに残す
//#define MATLAB_GRAPHICS_  //MATLABを起動し，Logをプロットする

using namespace std;

struct Capture
{
	bool flag;//メインループ実行フラグ
	unsigned int pic_cnt;//処理カウンター
	basler cam;//Baslerクラス	
	cv::Mat in_img;//光切断法入力画像
	cv::Mat out_img;//光切断画像処理出力画像
	cv::Mat bar_img;//棒に映るレーザーの検出用画像
	float rob_ini_height = 30.0f;//mm
	float height_interval = 10.0f;//mm
	float height = rob_ini_height;//mm
	vector<double> CoGs;//行ごとの輝度重心座標
	vector<double> barPoint;//Reference棒に映る点の重心
	vector<vector<double>> IdealPixs;//歪み補正ピクセル座標
	vector<vector<double>> barIdealPixs;//棒上の歪み補正ピクセル座標
	vector<double> lambdas;//直線の式の倍率
	vector<double> Heights_cam;//高度@カメラ座標系
	vector<double> Heights_world;//高度@World座標系
	vector<cv::Mat> Worlds;//World座標系
	vector<int> row_num;
	bool max_h_flg = false;
	//魚眼レンズ内部パラメーター
	vector<double> map_coefficient;
	vector<double> stretch_mat;
	vector<double> distortion;
	double laser_plane[3];
	vector<cv::Mat> Rots;
	vector<double> Rot_height;
	double barX[100];//ax+by+cz=1のReference barに映った輝点のX座標軍
	double planeA[100];//ax+by+cz=1のaの値群
	double planeB[100];//ax+by+cz=1のbの値群
	double planeC[100];//ax+by+cz=1のcの値群
	int barXint = 0;//Reference barに映った輝点のX座標群に該当する配列番号
	//Log格納用配列
	vector<double> Log_times;
	vector<vector<double>> CoGs_Logs;
	vector<vector<int>> Row_num_Logs;
	vector<vector<cv::Mat>> Worlds_Logs;
	vector<cv::Mat> Pictures;
	vector<bool> Processed;
};
//グローバル変数
double result, logtime ,det;
double moment, mass, cog,rowmass,colmass;
int rowcnt;
cv::Mat lhand, rhand,sol;
float timeout = 30.0;
cv::Mat campnt, wldpnt;
double hw;
int outputlog_num_max = 40;//出力する3次元座標の数
int outputlog_num_part = 20;//部分スキャンするときのレーザー本数
bool numchange_flg = false;//部分スキャンになったときに立てるフラグ
int numchange_timing = -1;//部分スキャンになった時のVector配列のIndex番号
int imgsavecnt = 0;//画像保存場所
int outputimgcnt = 0;//標準出力の画像列場所

//光切断法範囲指定
int LS_sta_row = 150;
int LS_end_row = 300;

//barに映るレーザー平面計算範囲指定
int PL_sta_row = 390;
int PL_end_row = 460;
cv::Rect bar_roi(60, 390, 300, 70);

/*
** シェーダオブジェクト
*/
static GLuint vertShader;
static GLuint fragShader;
static GLuint gl2Program;
GLFWwindow  *window;
//頂点オブジェクト
GLuint vao;
GLuint vbo;
//OpenGL内での変数
const int max_num = 150;
int logsize = 150;//750fpsの場合で何フレーム分出力するか
vector<float> pointlogs(max_num * 3 * logsize, 0);
vector<float> pos(max_num * 3, 0);
glm::mat4 mvp;
glm::mat4 View;
glm::mat4 Projection;
GLuint Matrix;
glm::vec3 campos;//カメラ位置座標ベクトル
vector<cv::Mat> gl_img_Logs;//OpenGL出力図の保存Vector
//View行列再計算時のパラメータ
float h_angle = 0;
float v_angle =  M_PI /6.0f;
float fov = 45.0f;
float speed = 3.0f;
float mousespeed = 0.005f;
float H_robot = 470;
const float H_camera = 600;


//Scan場所決定評価関数のパラメータ
double E[600];//評価関数値(-300<x<300の範囲)
double t[600] = { 0.0f };
double x_legm = -150;
double x_legp = 150;
double t_alpha = 0.02;
vector<vector<double>> E_Logs;
vector<int> argEmaxLogs;
vector<double> EvalTimeLogs;

//形状変化検出用パラメータ
vector<cv::Mat> Diffs;
cv::Mat oldimg,newimg;
cv::Mat diff;
cv::Mat thrmask = cv::Mat(480, 640, CV_8UC1, cv::Scalar::all(255));
cv::Mat	thrdiff;
int difframe = 50;//差分フレーム数
cv::Mat laserthr, laserthr2;//レーザー部分を抜くMask
cv::Moments M;//形状変化した点群のモーメント
cv::Rect roi(260, 170, 200, 200);
int meanval;//画像全体の平均輝度値
int detect_cnt_thr = 30;//形状変化検知のThreshold
int detection_num = 0;//形状変化と検出した回数
int max_detection_num = 5;//この数値回数分連続で形状変化と検出したらOK
double ldiff;//レーザー距離差分
vector<double> ldiffs;
double ltarget;//検出後の形状変化中心部のPixel座標
double lthreshold =50.0;//レーザー位置ずれの閾値
bool shapechange = 0;//形状変化検出フラグ(0：未検出，1：検出)
vector<float> Xlaser_log(outputlog_num_max*2, 0);//レーザーの位置X座標ログ@pix座標系
int mindiffno = 0;
vector<int> FrameNos;//newimgのフレーム番号
vector<int> Detect_pixcnt_log;//形状変化検出したピクセル点数のログ
vector<int> Detect_num_log;//形状変化と検知した回数のログ
int detect_cnt;//現在の検出処理カウンター番号
int detect_cnt_old;//一回前の検出処理カウンター番号
bool shapedetection_flg = false;


//ミラー制御用変数
RS232c mirror;
int mode = 0;
float mirrorV = 1.65;
float dV = 0;
float Vtarget = 0;
float delv;
float mirrordir = 1;
float max_angle = 80;
float max_laser_num = 50;
float part_angle = 20;
float part_laser_num = 30;
char buf_mirror[7];
vector<float> mirV_log;
vector<string> sendlog;
string temp_log;
unsigned int control_cnt = 0;

//Thread排他制御
cv::Mutex mutex,mutex2,flg;


//仮置き
float rei;


//時刻変数
LARGE_INTEGER freq, start,logend;

//プロトタイプ宣言
void TakePicture(Capture *cap, bool *flg);
int CalcHeights(Capture *cap);
void OutPutLogs(Capture *cap, bool *flg);
int readShaderSource(GLuint shader, const char *file);
int writepointcloud(Capture *cap, bool *flg);
void computeMatrices();
void ShapeChangeDetectionMultiFrame(Capture *cap, bool *flg);
void AxisToPoint(RS232c *axis, int *pointnum);//単軸ロボットのポイント指定移動

int main(int argc, char *argv[]) {
	LARGE_INTEGER end;
	if (!QueryPerformanceFrequency(&freq)) { return 0; }// 単位習得
	//カメラパラメーター
	int width = 640;
	int height = 480;
	float  fps = 750.0f;
	float gain = 1.0f;

	#ifdef ROBOT_MOVE_
	// 単軸ロボットのRS232接続
	RS232c axis;
	char buf[256];
	axis.Connect("COM3", 38400, ODDPARITY, 0, 0, 0, 20000, 20000);
	//単軸ロボットサーボ入力
	axis.Send("@SRVO1.1\r\n");
	axis.Read_CRLF(buf, 256);
	printf(buf);
	Sleep(1000);
	//スタート地点へ移動
	int startp = 30;
	AxisToPoint(&axis, &startp);
	#endif // ROBOT_MOVE_


	//キャプチャ用の構造体の宣言
	Capture cap;
	cap.in_img = (cv::Mat(height, width, CV_8UC1, cv::Scalar::all(255)));
	cap.flag = true;
	cap.pic_cnt = 0;
	cap.Processed.push_back(false);//先にFalseを追加しておく
	cap.cam.connect(0);
	cap.cam.setParam(paramTypeCamera::paramInt::WIDTH, width);
	cap.cam.setParam(paramTypeCamera::paramInt::HEIGHT, height);
	cap.CoGs.resize(height, 0);
	cap.cam.setParam(paramTypeBasler::FastMode::SensorReadoutModeFast);
	cap.cam.setParam(paramTypeBasler::Param::ExposureTime, 1280.0f);
	cap.cam.setParam(paramTypeCamera::paramFloat::FPS, fps);
	cap.cam.setParam(paramTypeCamera::paramFloat::GAIN, gain);
	//cap.cam.setParam(paramTypeBasler::AcquisitionMode::EnableAcquisitionFrameRate);//フレームレートを固定してパラメータ決める
	cap.cam.setParam(paramTypeBasler::AcquisitionMode::TriggerMode);//MBEDからTriggerが入ったときに撮像する
	cap.cam.setParam(paramTypeBasler::GrabStrategy::LatestOnlyFrame);//OnebyOne:撮像画像を捨てないで遅れてもバッファに貯めた1枚ずつ送る LatestOnlyFrame:常にバッファを更新して最新の画像を撮り続ける
																	 /*cap.cam.setParam(paramTypeBasler::CaptureType::BayerBGGrab);
																	 cap.cam.setParam(paramTypeBasler::CaptureType::ColorBGRGrab);*/
	cap.cam.setParam(paramTypeBasler::CaptureType::MonocroGrab);
	cap.cam.parameter_all_print();
	//Calibration結果の格納
	double a = 0, b = 0, c = 0, d = 0,h = 0;
	double R[9];
	FILE* fp;
	fp = fopen("cameraplanebarparamsinterpolate.csv", "r");
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
	fscanf(fp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,", &R[0], &R[1], &R[2], &R[3], &R[4], &R[5], &R[6], &R[7], &R[8]);
	cv::Mat Rs(cv::Size(3, 3), CV_64F, R);
	//Rs = Rs.inv();//MATLABからの出力時に転置をとった形になるのでここはいらなくなる
	cap.Rots.push_back(Rs.clone());
	//棒上の輝点のX座標群
	for (size_t i = 0; i < 100; i++){
		fscanf(fp, "%lf,", &cap.barX[i]);
	}
	//平面の式ax+by+cz=1のパラメータ
	for (size_t i = 0; i < 100; i++) {
		fscanf(fp, "%lf,", &cap.planeA[i]);
	}
	for (size_t i = 0; i < 100; i++) {
		fscanf(fp, "%lf,", &cap.planeB[i]);
	}
	for (size_t i = 0; i < 100; i++) {
		fscanf(fp, "%lf,", &cap.planeC[i]);
	}
	fclose(fp);

	//先に画像Vectorを用意する
	for (size_t i = 0; i < (int)(timeout*fps+100); i++)
	{
		cap.Pictures.push_back((cv::Mat(480, 640, CV_8UC1, cv::Scalar::all(255))));
		cap.Processed.push_back(false);
	}
	

	//detの計算をする
	det = 1.0 / (cap.stretch_mat[0] - cap.stretch_mat[1] * cap.stretch_mat[2]);//外でやる

	//カメラ起動
	cap.cam.start();

	//ミラー制御のためのMBEDマイコンへのRS232接続
	mirror.Connect("COM4", 115200, 8, NOPARITY);

	//スレッド作成
	bool flag = true;
	char picname[256] = "C:/Users/Mikihiro Ikura/Documents/GitHub/FisheyeCalibration/Photos/Laser_on/picture294mm.jpg";
	thread thr(TakePicture, &cap, &flag);//構造体に画像を保存するThread
	thread thr2(OutPutLogs, &cap, &flag);//現在の画像，計算高度をデバック出力するThread
	thread thr3(writepointcloud, &cap,&flag);//OpenGLで計算した点群を出力するThread
	thread thr4(ShapeChangeDetectionMultiFrame, &cap, &flag);//複数フレーム差分画像から移動体検知
#ifdef ROBOT_MOVE_
	int endp = 31;
	thread thr5(AxisToPoint, &axis, &endp);//ロボットを350mmへ下す
#endif // ROBOT_MOVE_

	
	
	Sleep(1);//threadのみ1ms実行し，画像を格納させる
	if (!QueryPerformanceCounter(&start)) { return 0; }

	//メインループ
	while (flag)
	{
		if (cap.Processed[cap.pic_cnt]){//写真があるときに
			CalcHeights(&cap);//高度計算
			if (!QueryPerformanceCounter(&end)) { return 0; }
			logtime = (double)(end.QuadPart - start.QuadPart) / freq.QuadPart;
			cap.Log_times.push_back(logtime);
			if (numchange_flg){//フラグがたったら
				numchange_timing = cap.Log_times.size() - 1;
				numchange_flg = false;
			}
			
		}
		
	}
	mirror.Send("q");
	if (thr.joinable())thr.join();
	if (thr2.joinable())thr2.join();
	if (thr3.joinable())thr3.join();
	if (thr4.joinable())thr4.join();
#ifdef ROBOT_MOVE_
	if (thr5.joinable())thr5.join();
#endif // ROBOT_MOVE_

	
	cap.cam.stop();
	cap.cam.disconnect();

	//ログ保存
	//結果保存用ファイルを日時付きで作成し開く
	FILE* fr;
	FILE* fr2;
	time_t timer;
	struct tm now;
	struct tm *local;
	timer = time(NULL);
	localtime_s(&now, &timer);
	char dir0[256];
	strftime(dir0, 256, "C:/Users/Mikihiro Ikura/Documents/GitHub/HighSpeedCamera/sample/ShapeMeasurement3D/results/%y%m%d", &now);
	_mkdir(dir0);
	char dir[256];
	strftime(dir, 256, "C:/Users/Mikihiro Ikura/Documents/GitHub/HighSpeedCamera/sample/ShapeMeasurement3D/results/%y%m%d/%H%M%S", &now);
	_mkdir(dir);
	char dir2[256];
	strftime(dir2, 256, "C:/Users/Mikihiro Ikura/Documents/GitHub/HighSpeedCamera/sample/ShapeMeasurement3D/results/%y%m%d/%H%M%S/Pictures", &now);
	_mkdir(dir2);
	strftime(dir2, 256, "C:/Users/Mikihiro Ikura/Documents/GitHub/HighSpeedCamera/sample/ShapeMeasurement3D/results/%y%m%d/%H%M%S/Pictures_diff", &now);
	_mkdir(dir2);
	char filename[256];
	char ShapeChange[256];
	strftime(filename, 256, "results/%y%m%d/%H%M%S/LS_result.csv", &now);
	strftime(ShapeChange, 256, "results/%y%m%d/%H%M%S/ShapeChange_result.csv", &now);
	fr = fopen(filename, "w");
	fr2 = fopen(ShapeChange, "w");

	//単軸ロボット切断
#ifdef ROBOT_MOVE_
	axis.Send("@SRVO0.1\r\n");
	axis.Read_CRLF(buf, 256);
#endif // ROBOT_MOVE_

	
	//cap内のLogを保存
	printf("saving Logs...   ");
	for (int i = 0; i < cap.Log_times.size(); i++){
		fprintf(fr, "%lf,", cap.Log_times[i]);
		/*for (int j = 0; j < cap.Row_num_Logs[i].size(); j++){
			fprintf(fr, "%d,", cap.Row_num_Logs[i][j]);
		}*/
		//fprintf(fr, "\n");
		if (i == numchange_timing) { 
			outputlog_num_max = outputlog_num_part;
			fprintf(fr, "%d,",1);
		}
		#ifdef ROBOT_MOVE_
		if (i==0){
			fprintf(fr, "%d,", 2);
		}
		#endif // ROBOT_MOVE_

		fprintf(fr, "\n");
		for (int j = 0; j < cap.Worlds_Logs[i].size(); j++) {
			fprintf(fr, "%lf,", cap.Worlds_Logs[i][j].at<double>(0, 0));
		}
		if (cap.Worlds_Logs[i].size()==0){
			fprintf(fr, "%lf,", 0);
		}
		
		fprintf(fr, "\n");
		for (int j = 0; j < cap.Worlds_Logs[i].size(); j++) {
			fprintf(fr, "%lf,", cap.Worlds_Logs[i][j].at<double>(0, 1));
		}
		if (cap.Worlds_Logs[i].size() == 0) {
			fprintf(fr, "%lf,", 0);
		}
		fprintf(fr, "\n");
		for (int j = 0; j < cap.Worlds_Logs[i].size(); j++) {
			fprintf(fr, "%lf,", cap.Worlds_Logs[i][j].at<double>(0, 2));
		}
		if (cap.Worlds_Logs[i].size() == 0) {
			fprintf(fr, "%lf,",0);
		}
		fprintf(fr, "\n");
	}
	for (int i = 0; i < FrameNos.size(); i++)//差分画像保存
	{
		fprintf(fr2, "%d,%d,%d\n", FrameNos[i],Detect_pixcnt_log[i],Detect_num_log[i]);
	}
	printf("Logs finish!\n");
	fclose(fr);
	fclose(fr2);

	//画像を保存
	#ifdef SAVE_IMG_
	printf("saving Imgs...   ");
	char videoname[256];
	char picturename[256];
	char picsubname[128];
	char diffname[256];
	char diffsubname[128];
	strftime(videoname, 256, "results/%y%m%d/%H%M%S/video.avi", &now);
	strftime(picsubname, 128, "results/%y%m%d/%H%M%S/Pictures/frame", &now);
	strftime(diffsubname, 128, "results/%y%m%d/%H%M%S/Pictures_diff/frame", &now);
	//cv::VideoWriter writer(videoname, cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 750.0, cv::Size(640, 480), false);//モノクロなのでFalse指定
	//if (!writer.isOpened()) { return -1; }
	for (int i = 0; i < imgsavecnt; i++) {//生画像保存
		//writer << cap.Pictures[i].clone();
		sprintf(picturename, "%s%d.png", picsubname, i);//jpg不可逆圧縮，png可逆圧縮
		cv::imwrite(picturename, cap.Pictures[i]);
	}
	printf("Imgs finish!\n");
	//writer.release();
	#endif // SAVE_IMG_

	//MATLABでLogの出力をする
	#ifdef MATLAB_GRAPHICS_
	printf("MATLAB Open...   ");
	int ret = 0;
	char matdir[256] = "C:\\Users\\Mikihiro Ikura\\Documents\\GitHub\\ShowLSLogs\\LS_result.csv";
	int cpy = CopyFileA(filename, matdir, FALSE);
	if (cpy) { printf("Copy success!   "); }
	else { printf("Copy failed...   "); }
	char matlabcmd[256] = "matlab -sd \"C:\\Users\\Mikihiro Ikura\\Documents\\GitHub\\ShowLSLogs\" -r ShowLSLogs -nosplash";
	ret = system(matlabcmd);
	if (ret == 0) { printf("MATLAB activate! \n"); }
	else{printf("error! \n");}
	#endif // MATLAB_GRAPHICS_

	printf("ALL FINALIZE ! \n");
	system("pause");
	return 0;
}

//thread内の関数
//ただ，capに写真を格納する
void TakePicture(Capture *cap, bool *flg) {
	cv::Mat temp = cv::Mat(480, 640, CV_8UC1, cv::Scalar::all(255));
	while (*flg) {
		//ここは排他制御しない
		cap->cam.captureFrame(temp.data);
		{
			cv::AutoLock lock(mutex);
			cap->Pictures[imgsavecnt] = temp.clone();
		}
		cap->Processed[imgsavecnt] = true;//今追加された写真の番号部分はTrueにする
		outputimgcnt = imgsavecnt;
		imgsavecnt++;
	}
}


//thread2内の関数
//標準出力で現在の写真，時刻と高度
void OutPutLogs(Capture *cap ,bool *flg) {
	while (1)
	{
		if (thrmask.data!=NULL&&outputimgcnt>3) {
			cv::imshow("img", cap->Pictures[outputimgcnt - 3]);
			cv::imshow("Diffs", thrmask);
		}
		int key = cv::waitKey(1);
		if (key == 'q') {
			*flg = false;
			break;
		}
		else if (key == 'd') {//ミラーのモード変更デバック用
			mirror.Send("d");
		}
		if (logtime > timeout) { 
			*flg = false;
			break;
		}
		if (cap->Worlds_Logs.size()>0){
			if (cap->Worlds_Logs[cap->Worlds_Logs.size() - 1].size() == 0) { printf("Time: ,%lf, s Bar: ,%lf, mm  Height: ,NONE    , mm  Detectcnt: %d, Continuous Detect: %d\n", logtime,cap->barX[cap->barXint], (int)M.m00 / 255,detection_num); }
			else {
				result = cap->Worlds_Logs[cap->Worlds_Logs.size() - 1][0].at<double>(0, 2);
				printf("Time: ,%lf, s Bar: ,%lf, mm  Height: ,%.4lf, mm Detectcnt: %d, Continuous Detect: %d\n", logtime, cap->barX[cap->barXint],result, (int)M.m00 / 255,detection_num);
			}
		}
		
	}
}

//main loop内での高度計算
int CalcHeights(Capture *cap) {
	//前回使用した配列のClear
	cap->CoGs.clear();
	cap->Heights_cam.clear();
	cap->Heights_world.clear();
	cap->IdealPixs.clear();
	cap->Worlds.clear();
	//cap->row_num.clear();
	cap->barPoint.clear();
	cap->barIdealPixs.clear();
	cap->lambdas.clear();
	shapedetection_flg = false;//排他制御，PicturesからCloneしたらアンロック
	//画像配列群から次の処理カウンターの番号の画像をin_imgに格納する
	{
		cv::AutoLock lock(mutex);
		cap->in_img = cap->Pictures[cap->pic_cnt].clone();
	}
	cap->pic_cnt++;	
	shapedetection_flg = true;//アンロック
	if (cap->in_img.data != NULL) {
		//輝度重心の計算
		cv::threshold(cap->in_img, cap->out_img, 150.0, 255.0, cv::THRESH_BINARY);
		cv::bitwise_and(cap->in_img, cap->out_img, cap->out_img);
		//Barに映るレーザー輝度重心の計算
		cv::threshold(cap->in_img(bar_roi), cap->bar_img, 70, 255.0, cv::THRESH_BINARY);
		cv::bitwise_and(cap->in_img(bar_roi), cap->bar_img, cap->bar_img);
		//Reference棒に映る点群の輝度重心をとる
		for (int i = 0; i < cap->bar_img.rows; i++) {
			moment = 0.0;
			mass = 0.0;
			for (int j = 0; j < cap->bar_img.cols; j++) {
				mass += cap->bar_img.data[i*cap->bar_img.cols + j];
				moment += j * cap->bar_img.data[i*cap->bar_img.cols + j];
			}
			if (mass <= 0) { cog = 0; }
			else {
				cog = moment / mass + bar_roi.x;
			}
			cap->barPoint.push_back(cog);
		}
		//LINEの部分の輝度重心を獲得する
		for (int i = LS_sta_row; i < LS_end_row; i++) {
			moment = 0.0;
			mass = 0.0;
			for (int j = 0; j < cap->out_img.cols; j++) {
				mass += cap->out_img.data[i*cap->out_img.cols + j];
				moment += j * cap->out_img.data[i*cap->out_img.cols + j];
			}
			if (mass > 0) {
				cog = moment / mass;
			}
			cap->CoGs.push_back(cog);
		}
		{
			cv::AutoLock coglock(mutex2);//Xlaser_logの排他制御
			if (cap->CoGs.size() == 0) { Xlaser_log.push_back(0); }
			else { Xlaser_log.push_back((float)cap->CoGs[cap->CoGs.size() / 2]); }
			Xlaser_log.erase(Xlaser_log.begin());
		}//アンロック
		cap->CoGs_Logs.push_back(cap->CoGs);//輝度重心のログ保存
		//輝度重心⇒理想ピクセル座標に変換
		vector<double> idpixs;
		double u, v, w;//次のfor内でも使う
		double ud, vd;
		double totalcog = 0;
		double cogcnt = 0;
		double barcog;
		//棒に映る輝点中心のX座標@pixel座標から，補完した点群を用いて平面パラメータを取得
		for (int i = 0; i < cap->barPoint.size(); i++)
		{
			if (cap->barPoint[i] == 0) { continue; }
			totalcog += cap->barPoint[i];
			cogcnt++;
		}
		barcog = totalcog / cogcnt;
		//もしレーザー平面が補間領内なら実行
		if (cap->barX[0]<barcog && barcog<cap->barX[99]) {
			cap->barXint = 0;
			for (int i = 0; i < 100; i++)
			{
				if (cap->barX[i]<barcog&&cap->barX[i + 1]>barcog) {
					cap->barXint = i;
					break;
				}
			}
			cap->laser_plane[0] = (cap->planeA[cap->barXint] * (cap->barX[cap->barXint + 1] - barcog) + cap->planeA[cap->barXint + 1] * (barcog - cap->barX[cap->barXint])) / (cap->barX[cap->barXint + 1] - cap->barX[cap->barXint]);
			cap->laser_plane[1] = (cap->planeB[cap->barXint] * (cap->barX[cap->barXint + 1] - barcog) + cap->planeB[cap->barXint + 1] * (barcog - cap->barX[cap->barXint])) / (cap->barX[cap->barXint + 1] - cap->barX[cap->barXint]);
			cap->laser_plane[2] = (cap->planeC[cap->barXint] * (cap->barX[cap->barXint + 1] - barcog) + cap->planeC[cap->barXint + 1] * (barcog - cap->barX[cap->barXint])) / (cap->barX[cap->barXint + 1] - cap->barX[cap->barXint]);

			//地面にあたるレーザーの理想ピクセル座標の計算
			for (int i = 0; i < cap->CoGs.size(); i++)
			{
				if (cap->CoGs[i] == 0) { continue; }
				ud = cap->CoGs[i] - cap->distortion[0];
				vd = (double)(i + LS_sta_row) - cap->distortion[1];
				u = det * (ud - cap->stretch_mat[1] * vd);
				v = det * (-cap->stretch_mat[2] * ud + cap->stretch_mat[0] * vd);
				idpixs.push_back(u);
				idpixs.push_back(v);
				cap->IdealPixs.push_back(idpixs);
				idpixs.clear();
			}

			//理想ピクセル座標⇒直線の式とレーザー平面の求解
			double phi, lambda, h;
			for (size_t i = 0; i < cap->IdealPixs.size(); i++)//0番目はReference棒への直線
			{
				u = cap->IdealPixs[i][0];
				v = cap->IdealPixs[i][1];
				phi = hypot(u, v);
				w = cap->map_coefficient[0] + cap->map_coefficient[1] * pow(phi, 2)
					+ cap->map_coefficient[2] * pow(phi, 3) + cap->map_coefficient[3] * pow(phi, 4);
				lambda = 1 / (cap->laser_plane[0] * u + cap->laser_plane[1] * v + cap->laser_plane[2] * w);
				cap->lambdas.push_back(lambda);
				h = lambda * w;
				cap->Heights_cam.push_back(h);
			}

			//カメラ座標系中心のWorld座標に変換
			for (size_t i = 0; i < cap->IdealPixs.size(); i++)//0番目は棒に映る点
			{
				campnt = (cv::Mat_<double>(1, 3) << cap->lambdas[i] * cap->IdealPixs[i][0], cap->lambdas[i] * cap->IdealPixs[i][1], cap->Heights_cam[i]);
				wldpnt = campnt * cap->Rots[0];
				hw = wldpnt.clone().at<double>(0, 2);
				cap->Worlds.push_back(wldpnt.clone());
				cap->Heights_world.push_back(hw);
			}
		}

		cap->Worlds_Logs.push_back(cap->Worlds);
		//cap->Row_num_Logs.push_back(cap->row_num);

		////処理カウンターと処理判別の更新
		//cap->pic_cnt++;

	}
	
	return 0;
}

/*
** シェーダーのソースプログラムをメモリに読み込む
*/
int readShaderSource(GLuint shader, const char *file)
{
	FILE *fp;
	const GLchar *source;
	GLsizei length;
	int ret;

	/* ファイルを開く */
	fp = fopen(file, "rb");
	if (fp == NULL) {
		perror(file);
		return -1;
	}

	/* ファイルの末尾に移動し現在位置 (つまりファイルサイズ) を得る */
	fseek(fp, 0L, SEEK_END);
	length = ftell(fp);

	/* ファイルサイズのメモリを確保 */
	source = (GLchar *)malloc(length);
	if (source == NULL) {
		fprintf(stderr, "Could not allocate read buffer.\n");
		return -1;
	}

	/* ファイルを先頭から読み込む */
	fseek(fp, 0L, SEEK_SET);
	ret = fread((void *)source, 1, length, fp) != (size_t)length;
	fclose(fp);

	/* シェーダのソースプログラムのシェーダオブジェクトへの読み込み */
	if (ret)
		fprintf(stderr, "Could not read file: %s.\n", file);
	else
		glShaderSource(shader, 1, &source, &length);

	/* 確保したメモリの開放 */
	free((void *)source);

	return ret;
}

//OpenGLで点群描画する関数
int writepointcloud(Capture *cap, bool *flg) {
	if (glfwInit() == GL_FALSE)
	{
		std::cerr << "Can't initilize GLFW" << std::endl;
		return 1;
	}
	//開くOpenGLのバージョン指定
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	window = glfwCreateWindow(1024, 768, "SAMPLE", NULL, NULL);
	if (window == nullptr)
	{
		std::cerr << "Can't create GLFW window." << std::endl;
		glfwTerminate();
		return 1;
	}

	glfwMakeContextCurrent(window);

	//GLEWの初期化
	//MakeCOntextcurrentの後に行わないと失敗するらしい
	if (glewInit() != GLEW_OK)
	{
		std::cerr << "Can't initilize GLEW" << std::endl;
		return 1;
	}

	glClearColor(0.2f, 0.2f, 0.2f, 0.2f);

	//Shaderオブジェクト作成
	vertShader = glCreateShader(GL_VERTEX_SHADER);
	fragShader = glCreateShader(GL_FRAGMENT_SHADER);

	//ソースプログラム読み込み
	if (readShaderSource(vertShader, "3dpoint.vert")) exit(1);
	if (readShaderSource(fragShader, "3dpoint.frag")) exit(1);

	//Shaderコンパイル
	glCompileShader(vertShader);
	glCompileShader(fragShader);

	//プログラムオブジェクト作成
	gl2Program = glCreateProgram();
	glAttachShader(gl2Program, vertShader);
	glDeleteShader(vertShader);
	glAttachShader(gl2Program, fragShader);
	glDeleteShader(fragShader);

	//プログラムオブジェクトのリンク
	glBindAttribLocation(gl2Program, 0, "position");
	glBindFragDataLocation(gl2Program, 0, "gl_FragColor");
	glLinkProgram(gl2Program);

	//頂点配列オブジェクト

	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);

	//頂点バッファオブジェクト

	glGenBuffers(1, &vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, pointlogs.size()*4, nullptr, GL_DYNAMIC_DRAW);

	//Vertexshaderの参照
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);

	//頂点バッファオブジェクトの結合解除
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	Matrix = glGetUniformLocation(gl2Program, "MVP");

	//保存用Matファイル
	cv::Mat gl_img(768, 1024, CV_8UC3);
	
	
	while (glfwWindowShouldClose(window) == GL_FALSE && *flg)
	{
		vector<cv::Mat> worlds = cap->Worlds_Logs[cap->Worlds_Logs.size() - 1];
		glClear(GL_COLOR_BUFFER_BIT);

		//Shaderプログラム使用開始
		glUseProgram(gl2Program);

		//視点行列の更新
		computeMatrices();
		mvp = Projection * View;
		glUniformMatrix4fv(Matrix, 1, GL_FALSE, &mvp[0][0]);

		
		//描画する点群
		for (auto j = 0; j < worlds.size(); ++j)
		{
			pos[j * 3 + 0] = worlds[j].at<double>(0, 0);
			pos[j * 3 + 1] = worlds[j].at<double>(0, 1);
			pos[j * 3 + 2] = worlds[j].at<double>(0, 2);
		}

		pointlogs.insert(pointlogs.end(), pos.begin(), pos.end());
		pointlogs.erase(pointlogs.begin(),pointlogs.begin()+450);
		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		//glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(position), &position[0][0]);//更新
																		/*ここに描画*/
		glBufferSubData(GL_ARRAY_BUFFER, 0, pointlogs.size()*4,&pointlogs[0]);
		glBindVertexArray(vao);
		glDrawArrays(GL_POINTS, 0, max_num*logsize);
		glBindVertexArray(0);
		

		//フロントバッファとバックバッファの入れ変え
		glfwSwapBuffers(window);

		//OpenGLのバッファをcv::mat配列に保存
		glReadBuffer(GL_BACK);
		glReadPixels(0, 0, 1024, 768, GL_BGR, GL_UNSIGNED_BYTE, gl_img.data);
		cv::flip(gl_img, gl_img, 0);
		/*cv::imshow("gl_img",gl_img);
		cv::waitKey(1);*/
		gl_img_Logs.push_back(gl_img.clone());//更新時間は光切断法と一致していない
	}
	glDeleteVertexArrays(1, &vao);
	glDeleteBuffers(1, &vbo);

	glfwTerminate();
}

void computeMatrices() {
	static double lastTime = glfwGetTime();//はじめだけ
	double currentTime = glfwGetTime();//毎回
	float deltaT = float(currentTime - lastTime);

	//Mouse位置から見ている方向を変更
	double xp, yp;
	glfwGetCursorPos(window, &xp, &yp);
	//glfwSetCursorPos(window, 1024/2, 768/2);//マウス位置リセット
	//printf("xp: %f, yp: %f \n",xp,yp);

	//カメラ位置ベクトル更新
	campos = glm::vec3(0, -400, H_robot-H_camera);

	//方向ベクトル更新
	/*h_angle += mousespeed * float(1024 / 2 - xp);
	v_angle += mousespeed * float(768 / 2 - yp);*/
	glm::vec3 direction(
		cos(v_angle)*sin(h_angle),
		sin(v_angle),
		cos(v_angle)*cos(h_angle)
	);
	//右ベクトル更新
	glm::vec3 right = glm::vec3(
		sin(h_angle - 3.14f / 2.0f),
		0,
		cos(h_angle - 3.14f / 2.0f)
	);
	//上ベクトル
	glm::vec3 up = glm::cross(right, direction);


	//矢印キー分だけカメラ位置変更
	// Move forward
	if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) {
		campos += direction * deltaT * speed;
	}
	// Move backward
	if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) {
		campos -= direction * deltaT * speed;
	}
	// Strafe right
	if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) {
		campos += right * deltaT * speed;
	}
	// Strafe left
	if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) {
		campos -= right * deltaT * speed;
	}

	//射影行列の更新
	Projection = glm::perspective(glm::radians(fov), 4.0f / 3.0f, 0.1f, 1000.0f);

	//カメラ行列の更新
	View = glm::lookAt(
		campos,
		campos+direction,
		up
	);
	
	lastTime = currentTime;//時間更新
}



//連続difframe枚分のフレーム画像の差分画像から形状変化を検出
void ShapeChangeDetectionMultiFrame(Capture *cap,bool *flg) {
	while (*flg&&!shapechange)
	{
		
		if(shapedetection_flg){
			detect_cnt = cap->pic_cnt;
			if (detect_cnt > difframe) {//difframe分だけ処理カウンタが進んだ
				oldimg = cap->Pictures[detect_cnt - difframe].clone();
				newimg = cap->Pictures[detect_cnt - 1].clone();
			}
		}		
		if (oldimg.data == NULL || newimg.data == NULL || detect_cnt <= detect_cnt_old) { continue; }
		detect_cnt_old = detect_cnt;
		//差分画像計算
		diff = abs(newimg - oldimg);

		//形状変化部分を差分画像から検出
		cv::threshold(oldimg, laserthr, 60, 255, cv::THRESH_BINARY);
		cv::threshold(newimg, laserthr2, 60, 255, cv::THRESH_BINARY);//レーザーが光っているところはThresholdかける
		cv::bitwise_or(laserthr, laserthr2, laserthr);
		cv::GaussianBlur(laserthr, laserthr, cv::Size(51, 17), 0);
		cv::threshold(laserthr(roi), laserthr, 0, 255, cv::THRESH_BINARY);
		cv::bitwise_not(laserthr, laserthr);
		meanval = (int)mean(diff)[0];//差分画像の平均画素値計算
		cv::bitwise_and(diff(roi), laserthr, thrmask);//レーザーが映ったところは消去
		cv::threshold(thrmask, thrmask, 15.0+meanval, 255.0, cv::THRESH_BINARY);//15+画素値平均値の閾値超えを255に
		
			

		//形状変化検出画像の部分から0次モーメント(Pixel数)計算
		M = cv::moments(thrmask);
		if ((int)M.m00/255 > detect_cnt_thr && !(M.nu02>100 * M.nu20))//閾値以上の点数が形状変化点だったら
		{
			detection_num++;
			if (detection_num > max_detection_num) {
				//形状変化検出⇒レーザー移動
				//検出点群の重心のX方向@pix座標系計算
				//レーザー中心を形状変化中心に移動
				ltarget = M.m10 / M.m00 + roi.x;//x軸1次モーメント/0次モーメント=x軸重心
				{
					cv::AutoLock xlaserlock(mutex2);//Xlaser_logの排他制御
					float minldiff = 1000;
					for (int i = 0; i < Xlaser_log.size(); i++)
					{
						ldiff = abs(Xlaser_log[i] - ltarget);
						if (minldiff > ldiff) {
							minldiff = ldiff;
							mindiffno = i;
						}
					}
				}//アンロック
				unsigned char frameno = (unsigned char)mindiffno;
				mirror.Send("l");//フレーム番号送信前のフラグ
				mirror.Send_CHAR(frameno);//フレーム番号送信
				numchange_flg = true;//ログ出力のレーザー本数の変更フラグ
				shapechange = 1;//形状変化検出フラグ更新⇒終了
			}
		}
		else{
			detection_num = 0;
		}
		//ログ保存
		FrameNos.push_back(detect_cnt_old - 1);
		Detect_pixcnt_log.push_back((int)M.m00 / 255);
		Detect_num_log.push_back(detection_num);
	}
}


//単軸ロボットのポイント指定移動
void AxisToPoint(RS232c *axis, int *pointnum) {
	char command[256] = { '\0' };
	char buf[256];
	int rob_height;
	//axis->Read_CRLF(buf, 256);
	//printf(buf);//RUN
	sprintf(command, "@START%d.1\r\n", *pointnum);
	axis->Send(command);
	axis->Read_CRLF(buf, 256);
	//printf(buf);//RUN
	Sleep(5000);//この時間以内に動き切る
	axis->Read_CRLF(buf, 256);
	//printf(buf);//END
	//Sleep(1000);
	//axis.Send("@?D0.1\r\n");
	//axis.Read_CRLF(buf, 256);
	//printf(buf);//D0.1=~~
	//			/*axis.Send("@?D0.1\r\n");
	//			axis.Read_CRLF(buf, 256);
	//			printf(buf);*/
	//sscanf(buf, "D0.1=%d\r\n", &rob_height);
	//cap.height = cap.rob_ini_height + (float)rob_height * 0.01;
}
