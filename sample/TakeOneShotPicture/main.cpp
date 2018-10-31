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

//�v���g�^�C�v�錾
void TakePicture(Capture *cap, bool *flg);

int main() {
	//�J�����p�����[�^
	int width = 640;
	int height = 480;
	float  fps = 750.0f;
	float gain = 1.0f;

	//�e��p�����[�^
	int counter = 0;
	string save_dir = "Photos/picure";
	int num_pic = 1;
	vector<cv::Mat> Pictures;
	vector<float> RobotHeights;
	cv::Mat blank = (cv::Mat(height, width, CV_8UC1, cv::Scalar::all(255)));;


	//�L���v�`���p�̍\���̂̐錾
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
	cap.cam.setParam(paramTypeBasler::AcquisitionMode::EnableAcquisitionFrameRate);//�t���[�����[�g���Œ肵�ăp�����[�^���߂�
	cap.cam.setParam(paramTypeBasler::GrabStrategy::LatestOnlyFrame);//OnebyOne:�B���摜���̂ĂȂ��Œx��Ă��o�b�t�@�ɒ��߂�1�������� LatestOnlyFrame:��Ƀo�b�t�@���X�V���čŐV�̉摜���B�葱����
																	 /*cap.cam.setParam(paramTypeBasler::CaptureType::BayerBGGrab);
																	 cap.cam.setParam(paramTypeBasler::CaptureType::ColorBGRGrab);*/
	cap.cam.setParam(paramTypeBasler::CaptureType::MonocroGrab);
	cap.cam.parameter_all_print();
	//�J�����N��
	cap.cam.start();

	bool flag = true;
	thread thr(TakePicture, &cap, &flag);

	bool on_flag = false;

	while (1) {
		cv::imshow("img", cap.in_img);
		int key = cv::waitKey(1);
		if (key == 'q')break;
		else if (key == 'p') { Pictures.push_back(cap.in_img.clone()); }
	}
	//�摜�̕ۑ�
	for (int i = 0; i < Pictures.size(); i++)
	{
		cv::imwrite(save_dir + "No" + to_string(int(i)) + ".jpg", Pictures[i]);
	}
	flag = false;
	if (thr.joinable())thr.join();

	cap.cam.stop();
	cap.cam.disconnect();

	

	return 0;
}

//thread���̊֐�
void TakePicture(Capture *cap, bool *flg) {
	while (*flg) {
		cap->cam.captureFrame(cap->in_img.data);
	}
}
