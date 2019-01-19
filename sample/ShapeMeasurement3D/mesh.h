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
** �V�F�[�_�I�u�W�F�N�g
*/
static GLuint vertShader;
static GLuint fragShader;
static GLuint gl2Program;

struct Capture
{
	bool flag;//���C�����[�v���s�t���O
	unsigned int pic_cnt;//�����J�E���^�[
	basler cam;//Basler�N���X
	cv::Mat in_img;//���ؒf�@���͉摜
	cv::Mat out_img;//���ؒf�摜�����o�͉摜
	float rob_ini_height = 30.0f;//mm
	float height_interval = 10.0f;//mm
	float height = rob_ini_height;//mm
	vector<double> CoGs;//�s���Ƃ̋P�x�d�S���W
	double barPoint[2];//Reference�_�ɉf��_�̏d�S
	vector<vector<double>> IdealPixs;//�c�ݕ␳�s�N�Z�����W
	double barIdealPixs[2];//�_��̘c�ݕ␳�s�N�Z�����W
	vector<double> lambdas;//�����̎��̔{��
	vector<double> Heights_cam;//���x@�J�������W�n
	vector<double> Heights_world;//���x@World���W�n
	vector<cv::Mat> Worlds;//World���W�n
	vector<int> row_num;
	bool max_h_flg = false;
	//���჌���Y�����p�����[�^�[
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
	//Log�i�[�p�z��
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