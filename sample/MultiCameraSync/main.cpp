
#include <thread>
#include <windows.h>
#include <iostream>

#include <opencv2/opencv.hpp>
#ifdef _DEBUG
#define CAMERA_EXT "d.lib"
#else
#define CAMERA_EXT ".lib"
#endif

#define STR_EXP(__A) #__A
#define STR(__A) STR_EXP(__A)
#define CV_VER STR(CV_VERSION_MAJOR) STR(CV_VERSION_MINOR) STR(CV_VERSION_REVISION)
#pragma comment(lib, "opencv_world" CV_VER CAMERA_EXT)

#define USE_IDPEXPRESS
#define USE_BASLER
#define USE_XIMEA
#include <HSC/CameraFunc.hpp>

int main() {
	HANDLE sh = CreateEvent(NULL, TRUE, FALSE, "EVENT");

	Camera* cam1 = getCamera(cameraType::IDPEXPRESS);
	Camera* cam2 = getCamera(cameraType::XIMEA);
	int width = 512;
	int height = 512;
	float  fps = 50.0f;
	float gain = 1.0f;
	auto img1 = (cv::Mat(height, width, CV_8UC1, cv::Scalar::all(255)));
	auto img2 = (cv::Mat(488, 648, CV_8UC1, cv::Scalar::all(255)));

	cam1->connect(0);
	cam1->setParam(paramTypeCamera::paramInt::WIDTH, width);
	cam1->setParam(paramTypeCamera::paramInt::HEIGHT, height);
	cam1->setParam(paramTypeCamera::paramFloat::FPS, fps);
	cam1->setParam(paramTypeCamera::paramFloat::GAIN, gain);
	cam1->setParam(paramTypeIdpExpress::AcquisitionMode::EnableAcquisitionFrameRate);
	cam1->start();

	cam2->connect(0);
	cam2->setParam(paramTypeCamera::paramInt::WIDTH, 648);
	cam2->setParam(paramTypeCamera::paramInt::HEIGHT, 488);
	cam2->setParam(paramTypeCamera::paramFloat::FPS, fps);
	cam2->setParam(paramTypeCamera::paramFloat::GAIN, gain);
	cam2->setParam(paramTypeXimea::AcquisitionMode::EnableAcquisitionFrameRate);
	cam2->setParam(paramTypeXimea::CaptureType::BayerGrab);
	cam2->start();

	bool flag = true;
	std::thread thr1([&] {

		HANDLE h = OpenEvent(EVENT_ALL_ACCESS, FALSE, "EVENT");
		WaitForSingleObject(h, INFINITE);

		while (flag) {
			cam1->captureFrame(img1.data);
		}
	});


	std::thread thr2([&] {
		HANDLE h = OpenEvent(EVENT_ALL_ACCESS, FALSE, "EVENT");
		WaitForSingleObject(h, INFINITE);

		while (flag) {
			cam2->captureFrame(img2.data);
		}
	});


	getchar();
	SetEvent(sh);

	while (1) {
		cv::imshow("img1", img1);
		cv::imshow("img2", img2);
		int key = cv::waitKey(1);
		if (key == 'q')break;
	}
	flag = false;
	if (thr1.joinable())thr1.join();
	if (thr2.joinable())thr2.join();

	delete cam1;
	delete cam2;
}