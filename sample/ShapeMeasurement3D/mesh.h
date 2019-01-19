#pragma once

#include <cstdio>
#include <iostream>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <vector>

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

#include <HSC/baslerClass.hpp>
#pragma comment(lib, "BaslerLib" CAMERA_EXT)

#pragma warning(disable:4996)

using namespace std;

/*
** シェーダオブジェクト
*/
static GLuint vertShader;
static GLuint fragShader;
static GLuint gl2Program;

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
	double barPoint[2];//Reference棒に映る点の重心
	vector<vector<double>> IdealPixs;//歪み補正ピクセル座標
	double barIdealPixs[2];//棒上の歪み補正ピクセル座標
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
	cv::Vec3d common_line;
	cv::Vec3d common_lineP;
	cv::Vec3d reference_line;
	cv::Vec3d reference_lineP;
	cv::Vec3d barPointCam;
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

int readShaderSource(GLuint shader, const char *file);
int initmesh();
void makemesh(Capture *cap, bool *flg);
void endmesh();