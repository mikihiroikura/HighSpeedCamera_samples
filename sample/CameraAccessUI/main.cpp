#define _CRT_SECURE_NO_WARNINGS
#include <thread>
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

#define USE_XIMEA
#include <HSC/CameraUI.hpp>

int main() {

	CameraUI cam(CameraUI::cameraType::XIMEA);
	int width = 640;
	int height = 480;
	float  fps = 250.0f;
	float gain = 1.0f;
	auto img = (cv::Mat(height, width, CV_8UC3, cv::Scalar::all(255)));

	cam.connect(0);
	cam.setParam(paramTypeCamera::paramInt::WIDTH, width);
	cam.setParam(paramTypeCamera::paramInt::HEIGHT, height);
	cam.setParam(paramTypeCamera::paramFloat::FPS, fps);
	cam.setParam(paramTypeCamera::paramFloat::GAIN, gain);
	cam.setParam(paramTypeXimea::AcquisitionMode::EnableAcquisitionFrameRate);
	cam.setParam(paramTypeXimea::CaptureType::ColorGrab);
	cam.start();

	bool flag = true;
	std::thread thr([&] {
		while (flag) {
			cam.captureFrame(img.data);
		}
	});

	while (1) {
		cv::imshow("img", img);
		int key = cv::waitKey(1);
		if (key == 'q')break;
	}
	flag = false;
	if (thr.joinable())thr.join();

	cam.stop();
	cam.disconnect();

	return 0;
}