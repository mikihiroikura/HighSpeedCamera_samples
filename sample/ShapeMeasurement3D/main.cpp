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
//#define SAVE_IMG_         //画像と動画をLogに残す
//#define MATLAB_GRAPHICS_  //MATLABを起動し，Logをプロットする

using namespace std;

struct Capture
{
	bool flag;//メインループ実行フラグ
	unsigned int pic_cnt;//処理カウンター
	basler cam;//Baslerクラス
	cv::Mat in_img;//光切断法入力画像
	cv::Mat out_img;//光切断画像処理出力画像
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
	cv::Vec3d barPlane;
	cv::Vec3d lightPoint;
	cv::Vec3d barLineCam;
	cv::Vec3d barLineP;
	double laser_plane[3];
	vector<cv::Mat> Rots;
	vector<double> Rot_height;
	//Log格納用配列
	vector<double> Log_times;
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
double timeout = 60.0;
cv::Mat campnt, wldpnt;
double hw;
cv::PCA pca;

//toriaezu
double toria = 0;

//光切断法範囲指定
int LS_sta_row = 150;
int LS_end_row = 300;
//平面計算範囲指定
int PL_sta_row = 380;
int PL_end_row = 400;

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

const int max_num = 150;
GLfloat position[max_num][3];//numは定数でないといけないので注意
glm::mat4 mvp;
glm::mat4 View;
glm::mat4 Projection;
GLuint Matrix;
glm::vec3 campos = glm::vec3(-200, -210, 460);

float h_angle = 3.14f;
float v_angle = 0.0f;
float fov = 60.0f;
float speed = 3.0f;
float mousespeed = 0.005f;

//プロトタイプ宣言
void TakePicture(Capture *cap, bool *flg);
void CalcHeights(Capture *cap);
void OutPutLogs(Capture *cap, bool *flg);
int readShaderSource(GLuint shader, const char *file);
int writepointcloud(Capture *cap, bool *flg);
void GetPicture(Capture *cap, bool *flg, char * filename);
void computeMatrices();

int main(int argc, char *argv[]) {
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
	fp = fopen("cameramultiplanebarparams.csv", "r");
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
	//棒上の平面の式パラメータz=ax+by+c
	fscanf(fp, "%lf,%lf,%lf,", &a, &b, &c);
	cap.barPlane = cv::Vec3d(a, b, c);
	//輝点の座標
	fscanf(fp, "%lf,%lf,%lf,", &a, &b, &c);
	cap.lightPoint = cv::Vec3d(a, b, c);
	fclose(fp);

	//detの計算をする
	det = 1.0 / (cap.stretch_mat[0] - cap.stretch_mat[1] * cap.stretch_mat[2]);//外でやる

	

	//カメラ起動
	cap.cam.start();

	//結果保存用ファイルを日時付きで作成し開く
	FILE* fr;
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
	char filename[256];
	strftime(filename, 256, "results/%y%m%d/%H%M%S/LS_result.csv", &now);
	fr = fopen(filename, "w");
	
	//toriaezu
	vector<cv::Mat> temp;
	temp.push_back(cv::Mat(1, 3, CV_64F, cv::Scalar::all(255)));
	cap.Worlds_Logs.push_back(temp);

	//スレッド作成
	bool flag = true;
	char picname[256] = "C:/Users/Mikihiro Ikura/Documents/GitHub/FisheyeCalibration/Photos/Laser_on/picture294mm.jpg";
	thread thr(TakePicture, &cap, &flag);
	thread thr2(OutPutLogs, &cap, &flag);
	//thread thr3(writepointcloud, &cap,&flag);
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
			
		}
		
	}
	if (thr.joinable())thr.join();
	if (thr2.joinable())thr2.join();
	//if (thr3.joinable())thr3.join();

	cap.cam.stop();
	cap.cam.disconnect();

	//cap内のLogを保存
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

	//画像を動画保存
	#ifdef SAVE_IMG_
	printf("saving Imgs...   ");
	char videoname[256];
	char picname[256];
	char picsubname[128];
	strftime(videoname, 256, "results/%y%m%d/%H%M%S/video.avi", &now);
	strftime(picsubname, 128, "results/%y%m%d/%H%M%S/Pictures/frame", &now);
	cv::VideoWriter writer(videoname, cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 750.0, cv::Size(640, 480), false);//モノクロなのでFalse指定
	if (!writer.isOpened()) { return -1; }
	for (int i = 0; i < cap.Pictures.size(); i++) {
		writer << cap.Pictures[i].clone();
		sprintf(picname, "%s%d.jpg", picsubname, i);//jpg不可逆圧縮，png可逆圧縮
		cv::imwrite(picname, cap.Pictures[i]);
	}
	printf("Imgs finish!\n");
	writer.release();
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
	while (*flg) {
		cap->Pictures.push_back((cv::Mat(480, 640, CV_8UC1, cv::Scalar::all(255))));
		cap->cam.captureFrame(cap->Pictures[cap->Pictures.size()-1].data);
		cap->Processed[cap->Pictures.size() - 1] = true;//今追加された写真の番号部分はTrueにする
		cap->Processed.push_back(false);//最新番号の次の番号部分はFalseとしておく
	}
}

//thread内関数
//指定したディレクトリの画像を入力する
void GetPicture(Capture *cap, bool *flg,char *filename) {
	cv::Mat img = cv::imread(filename,cv::IMREAD_GRAYSCALE);
	while (*flg)
	{
		cap->Pictures.push_back(img);
		cap->Processed[cap->Pictures.size() - 1] = true;//今追加された写真の番号部分はTrueにする
		cap->Processed.push_back(false);//最新番号の次の番号部分はFalseとしておく
	}
}

//thread2内の関数
//標準出力で現在の写真，時刻と高度
void OutPutLogs(Capture *cap ,bool *flg) {
	while (1)
	{
		cv::imshow("img", cap->in_img);
		int key = cv::waitKey(1);
		if (key == 'q') {
			*flg = false;
			break;
		}
		if (logtime > timeout) { 
			*flg = false;
			break;
		}
		if (cap->Worlds_Logs.size()>0){
			if (cap->Worlds_Logs[cap->Worlds_Logs.size() - 1].size() == 0) { printf("Time: ,%lf, s  Height: ,NONE,\n", logtime); }
			else {
				result = cap->Worlds_Logs[cap->Worlds_Logs.size() - 1][0].at<double>(0, 2);
				printf("Time: ,%lf, s  Height: ,%lf, mm\n", logtime, result);
			}
		}
		
	}
}

//main loop内での高度計算
void CalcHeights(Capture *cap) {
	//前回使用した配列のClear
	cap->CoGs.clear();
	cap->Heights_cam.clear();
	cap->Heights_world.clear();
	cap->IdealPixs.clear();
	cap->Worlds.clear();
	cap->row_num.clear();
	cap->barPoint.clear();
	cap->barIdealPixs.clear();
	//画像配列群から次の処理カウンターの番号の画像をin_imgに格納する
	cap->in_img = cap->Pictures[cap->pic_cnt];	
	//輝度重心の計算
	cv::threshold(cap->in_img, cap->out_img, 150.0, 255.0, cv::THRESH_BINARY);
	cv::bitwise_and(cap->in_img, cap->out_img, cap->out_img);
	//Reference棒に映る点群の輝度重心をとる
	for (int i = PL_sta_row; i < PL_end_row; i++) {
		moment = 0.0;
		mass = 0.0;
		for (int j = 0; j < cap->out_img.cols; j++) {
			mass += cap->out_img.data[i*cap->out_img.cols + j];
			moment += j * cap->out_img.data[i*cap->out_img.cols + j];
		}
		if (mass <= 0) { cog = 0; }
		else {
			cog = moment / mass;
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
		if (mass <= 0) { cog = 0; }
		else {
			cog = moment / mass;
			cap->row_num.push_back(i);
		}
		cap->CoGs.push_back(cog);
	}
	//輝度重心⇒理想ピクセル座標に変換
	vector<double> idpixs;
	double u, v, w;//次のfor内でも使う
	double ud, vd;
	for (int i = 0; i < cap->barPoint.size(); i++)
	{
		if (cap->barPoint[i] == 0) { continue; }
		ud = cap->barPoint[i] - cap->distortion[0];
		vd = (double)(i + PL_sta_row) - cap->distortion[1];
		u = det * (ud - cap->stretch_mat[1] * vd);
		v = det * (-cap->stretch_mat[2] * ud + cap->stretch_mat[0] * vd);
		idpixs.push_back(u);
		idpixs.push_back(v);
		cap->barIdealPixs.push_back(idpixs);
		idpixs.clear();
	}
	//閾値で切った画像が真っ暗の時(barIdealPixs.size()=0)の時は何もしない
	if (cap->barIdealPixs.size() > 0)
	{
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

		//棒上の理想ピクセル座標群⇒直線+輝点⇒レーザー平面
		double phi, lambda, h;
		cv::Mat src(cap->barIdealPixs.size(), 3, CV_32FC1);
		//理想ピクセル座標群+棒上平面の式⇒直線@Camera座標に最適化
		for (size_t i = 0; i < cap->barIdealPixs.size(); i++)
		{
			u = cap->barIdealPixs[i][0];
			v = cap->barIdealPixs[i][1];
			phi = hypot(u, v);
			w = cap->map_coefficient[0] + cap->map_coefficient[1] * phi*phi
				+ cap->map_coefficient[2] * phi*phi*phi + cap->map_coefficient[3] * phi*phi*phi*phi;
			lambda = -cap->barPlane[2] / (cap->barPlane[0] * u + cap->barPlane[1] * v - w);
			((float*)src.data)[i*src.cols + 0] = lambda * u;
			((float*)src.data)[i*src.cols + 1] = lambda * v;
			((float*)src.data)[i*src.cols + 2] = lambda * w;
			//printf("%f,%f,%f\n", lambda*u, lambda*v, lambda*w);
		}
		/*((float*)src.data)[cap->barIdealPixs.size()*src.cols + 0] = cap->lightPoint[0];
		((float*)src.data)[cap->barIdealPixs.size()*src.cols + 1] = cap->lightPoint[1];
		((float*)src.data)[cap->barIdealPixs.size()*src.cols + 2] = cap->lightPoint[2];*/
		//cv::PCA pca(src, cv::Mat(), CV_PCA_DATA_AS_ROW, 3);
		pca = pca(src, cv::Mat(), CV_PCA_DATA_AS_ROW, 3);
		cap->barLineCam = cv::Vec3d(pca.eigenvectors.at<float>(0, 0), pca.eigenvectors.at<float>(0, 1), pca.eigenvectors.at<float>(0, 2));
		cap->barLineP = cv::Vec3d(pca.mean.at<float>(0, 0), pca.mean.at<float>(0, 1), pca.mean.at<float>(0, 2));
		//輝点と棒上Laser直線を通る平面z=ax+by+cの計算
		cv::Vec3d ref = cap->barLineP - cap->lightPoint;
		cv::Vec3d norm = ref.cross(cap->barLineCam);
		norm = -norm / norm[2];
		double c = -norm.dot(cap->lightPoint);
		cap->laser_plane[0] = norm[0];
		cap->laser_plane[1] = norm[1];
		cap->laser_plane[2] = c;
		/*cap->barLineCam = cv::Vec3d(pca.eigenvectors.at<float>(0, 0), pca.eigenvectors.at<float>(0, 1), pca.eigenvectors.at<float>(0, 2));
		cap->barLineP = cv::Vec3d(pca.eigenvectors.at<float>(1, 0), pca.eigenvectors.at<float>(1, 1), pca.eigenvectors.at<float>(1, 2));
		cv::Vec3d norm = cap->barLineP.cross(cap->barLineCam);
		norm = -norm / norm[2];
		double c = -norm.dot(cap->lightPoint);
		cap->laser_plane[0] = norm[0];
		cap->laser_plane[1] = norm[1];
		cap->laser_plane[2] = c;*/

		//理想ピクセル座標⇒直線の式とレーザー平面の求解
		for (size_t i = 0; i < cap->IdealPixs.size(); i++)//0番目はReference棒への直線
		{
			u = cap->IdealPixs[i][0];
			v = cap->IdealPixs[i][1];
			phi = hypot(u, v);
			w = cap->map_coefficient[0] + cap->map_coefficient[1] * pow(phi, 2)
				+ cap->map_coefficient[2] * pow(phi, 3) + cap->map_coefficient[3] * pow(phi, 4);
			lambda = -cap->laser_plane[2] / (cap->laser_plane[0] * u + cap->laser_plane[1] * v - w);
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
		

		////とりあえず
		//vector<cv::Mat> temp;
		//temp.push_back(cv::Mat(1, 3, CV_64F, cv::Scalar::all(toria)));
		//toria += 0.1;
		//if (toria>4)
		//{
		//	toria = 0;
		//}
		//cap->Worlds_Logs.push_back(temp);
	}
	cap->Worlds_Logs.push_back(cap->Worlds);
	cap->Row_num_Logs.push_back(cap->row_num);

	//処理カウンターと処理判別の更新
	cap->pic_cnt++;
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

	//点群初期化
	for (auto j = 0; j < max_num; ++j)
	{
		position[j][0] = 0.0f;
		position[j][1] = 0.0f;
		position[j][2] = 0.0f;
	}

	//頂点配列オブジェクト

	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);

	//頂点バッファオブジェクト

	glGenBuffers(1, &vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(position), nullptr, GL_DYNAMIC_DRAW);

	//Vertexshaderの参照
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);

	//頂点バッファオブジェクトの結合解除
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	// カメラ行列
	View = glm::lookAt(
		glm::vec3(0, 4, 5), // ワールド空間でカメラは(4,3,3)にあります。
		glm::vec3(0, 0, 0), // 原点を見ています。
		glm::vec3(0, 1, 0)  // 頭が上方向(0,-1,0にセットすると上下逆転します。)
	);
	glm::mat4 E = glm::mat4(1.0f);
	Projection = glm::perspective(glm::radians(fov), 4.0f / 3.0f, 0.1f, 100.0f);
	mvp = Projection * View;

	//campos = glm::vec3(0, 4, 5);

	Matrix = glGetUniformLocation(gl2Program, "MVP");

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
			position[j][0] = worlds[j].at<double>(0, 0);
			position[j][1] = worlds[j].at<double>(0, 1);
			position[j][2] = worlds[j].at<double>(0, 2);
		}

		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(position), position);//更新
																		/*ここに描画*/
		glBindVertexArray(vao);
		glDrawArrays(GL_POINTS, 0, max_num);
		glBindVertexArray(0);

		glfwSwapBuffers(window);
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
	glfwSetCursorPos(window, 1024/2, 768/2);//マウス位置リセット
	printf("xp: %f, yp: %f \n",xp,yp);

	//方向ベクトル更新
	h_angle += mousespeed * float(1024 / 2 - xp);
	v_angle += mousespeed * float(768 / 2 - yp);
	glm::vec3 direction(
		cos(v_angle)*sin(h_angle),
		sin(v_angle),
		cos(v_angle)*sin(h_angle)
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
	Projection = glm::perspective(glm::radians(fov), 4.0f / 3.0f, 0.1f, 100.0f);

	//カメラ行列の更新
	View = glm::lookAt(
		campos,
		campos + direction,
		up
	);
	
	//View = glm::lookAt(
	//	glm::vec3(0, 4, 5), // ワールド空間でカメラは(4,3,3)にあります。
	//	glm::vec3(0, 0, 0)+direction, // 原点を見ています。
	//	glm::vec3(0, 1, 0)  // 頭が上方向(0,-1,0にセットすると上下逆転します。)
	//);

	lastTime = currentTime;//時間更新
}