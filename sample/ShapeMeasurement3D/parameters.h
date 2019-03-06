//共通パラメータの宣言を行う
//定義はダメ

#pragma once
#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include <opencv2/opencv.hpp>
#include <HSC/baslerClass.hpp>
#include <vector>

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
#endif // !PARAMETERS_H_
