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

#ifdef _DEBUG
#define CAMERA_EXT "d.lib"
#else
#define CAMERA_EXT ".lib"
#endif

//opencv_world341(d).libの追加
#define STR_EXP(__A) #__A
#define STR(__A) STR_EXP(__A)
#define CV_VER STR(CV_VERSION_MAJOR) STR(CV_VERSION_MINOR) STR(CV_VERSION_REVISION)
#pragma comment(lib, "opencv_world" CV_VER CAMERA_EXT)

#define MAX_

#include <HSC/baslerClass.hpp>
#pragma comment(lib, "BaslerLib" CAMERA_EXT)

#pragma warning(disable:4996)

#include "motion_detect.h"
#include "graphics.h"
#include "parameters.h"


//DEFINE群
#define ROBOT_MOVE_//ロボットを動かす
#define SAVE_IMG_         //画像と動画をLogに残す
//#define MATLAB_GRAPHICS_  //MATLABを起動し，Logをプロットする

using namespace std;

//グローバル変数
double result, logtime, det;
double moment, mass, cog, rowmass, colmass;
int rowcnt;
cv::Mat lhand, rhand, sol;
float timeout = 30.0;
cv::Mat campnt, wldpnt;
double hw;
int outputlog_num_max = 40;//出力する3次元座標の数
int outputlog_num_part = 20;//部分スキャンするときのレーザー本数
bool numchange_flg = false;//部分スキャンになったときに立てるフラグ
int numchange_timing = -1;//部分スキャンになった時のVector配列のIndex番号
int imgsavecnt = 0;//画像保存場所
int outputimgcnt = 0;//標準出力の画像列場所
RS232c mirror;
vector<float> Xlaser_log(outputlog_num_max * 2, 0);//レーザーの位置X座標ログ@pix座標系
cv::Mat thrmask = cv::Mat(480, 640, CV_8UC1, cv::Scalar::all(255));
bool shapedetection_flg = false;
cv::Moments M;//形状変化した点群のモーメント
vector<int> FrameNos;//newimgのフレーム番号
vector<int> Detect_num_log;//形状変化と検知した回数のログ
int detection_num = 0;//形状変化と検出した回数
vector<int> Detect_pixcnt_log;//形状変化検出したピクセル点数のログ

//光切断法範囲指定
int LS_sta_row = 150;
int LS_end_row = 300;

//barに映るレーザー平面計算範囲指定
int PL_sta_row = 390;
int PL_end_row = 460;
cv::Rect bar_roi(60, 390, 300, 70);


//Thread排他制御
cv::Mutex mutex,mutex2,flg;

//時刻変数
LARGE_INTEGER freq, start,logend;

//プロトタイプ宣言
void TakePicture(Capture *cap, bool *flg);
int CalcHeights(Capture *cap);
void OutPutLogs(Capture *cap, bool *flg);
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
	axis.Connect("COM3", 38400, 8,ODDPARITY, 0, 0, 0, 20000, 20000);
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
	mirror.Connect("COM4", 115200, 8, NOPARITY,0,0,0,5000,20000);

	//スレッド作成
	bool flag = true;
	thread thr(TakePicture, &cap, &flag);//構造体に画像を保存するThread
	thread thr2(OutPutLogs, &cap, &flag);//現在の画像，計算高度をデバック出力するThread
	thread thr3(writepointcloud, &cap,&flag);//OpenGLで計算した点群を出力するThread
	thread thr4(ShapeChangeDetectionMultiFrame, &cap, &flag);//複数フレーム差分画像から移動体検知
#ifdef ROBOT_MOVE_
	int endp = 31;//動作指定番号
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
		sprintf(picturename, "%s%d.png", picsubname, i);//png可逆圧縮
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
	cap->IdealPixs.clear();
	cap->Worlds.clear();
	//cap->row_num.clear();
	cap->barPoint.clear();
	cap->barIdealPixs.clear();
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
			for (size_t i = 0; i < cap->IdealPixs.size(); i++)
			{
				u = cap->IdealPixs[i][0];
				v = cap->IdealPixs[i][1];
				phi = hypot(u, v);
				w = cap->map_coefficient[0] + cap->map_coefficient[1] * pow(phi, 2)
					+ cap->map_coefficient[2] * pow(phi, 3) + cap->map_coefficient[3] * pow(phi, 4);
				lambda = 1 / (cap->laser_plane[0] * u + cap->laser_plane[1] * v + cap->laser_plane[2] * w);
				campnt = (cv::Mat_<double>(1, 3) << lambda * u, lambda * v, lambda*w);//カメラ座標系位置
				wldpnt = campnt * cap->Rots[0];//回転行列をかけることでWorld座標系に直している
				cap->Worlds.push_back(wldpnt.clone());
			}
		}

		cap->Worlds_Logs.push_back(cap->Worlds);
		//cap->Row_num_Logs.push_back(cap->row_num);

	}
	
	return 0;
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
