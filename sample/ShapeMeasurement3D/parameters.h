//���ʃp�����[�^�̐錾���s��
//��`�̓_��

#pragma once
#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include <opencv2/opencv.hpp>
#include <HSC/baslerClass.hpp>
#include <vector>

using namespace std;

struct Capture
{
	bool flag;//���C�����[�v���s�t���O
	unsigned int pic_cnt;//�����J�E���^�[
	basler cam;//Basler�N���X	
	cv::Mat in_img;//���ؒf�@���͉摜
	cv::Mat out_img;//���ؒf�摜�����o�͉摜
	cv::Mat bar_img;//�_�ɉf�郌�[�U�[�̌��o�p�摜
	float rob_ini_height = 30.0f;//mm
	float height_interval = 10.0f;//mm
	float height = rob_ini_height;//mm
	vector<double> CoGs;//�s���Ƃ̋P�x�d�S���W
	vector<double> barPoint;//Reference�_�ɉf��_�̏d�S
	vector<vector<double>> IdealPixs;//�c�ݕ␳�s�N�Z�����W
	vector<vector<double>> barIdealPixs;//�_��̘c�ݕ␳�s�N�Z�����W
	vector<cv::Mat> Worlds;//World���W�n
	vector<int> row_num;
	bool max_h_flg = false;
	//���჌���Y�����p�����[�^�[
	vector<double> map_coefficient;
	vector<double> stretch_mat;
	vector<double> distortion;
	double laser_plane[3];
	vector<cv::Mat> Rots;
	vector<double> Rot_height;
	double barX[100];//ax+by+cz=1��Reference bar�ɉf�����P�_��X���W�R
	double planeA[100];//ax+by+cz=1��a�̒l�Q
	double planeB[100];//ax+by+cz=1��b�̒l�Q
	double planeC[100];//ax+by+cz=1��c�̒l�Q
	int barXint = 0;//Reference bar�ɉf�����P�_��X���W�Q�ɊY������z��ԍ�
					//Log�i�[�p�z��
	vector<double> Log_times;
	vector<vector<double>> CoGs_Logs;
	vector<vector<int>> Row_num_Logs;
	vector<vector<cv::Mat>> Worlds_Logs;
	vector<cv::Mat> Pictures;
	vector<bool> Processed;
};
#endif // !PARAMETERS_H_
