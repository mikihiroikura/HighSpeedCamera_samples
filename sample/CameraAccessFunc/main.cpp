
#include <opencv2/opencv.hpp>
#include <thread>

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
#include <HSC/CameraFunc.hpp>

int main() {
	Camera* cam = getCamera(cameraType::IDPEXPRESS);

	int width = 512;
	int height = 512;
	float  fps = 125.0f;
	float gain = 3.0f;

	auto img = (cv::Mat(height, width, CV_8UC1, cv::Scalar::all(255)));
	auto img2 = (cv::Mat(height, width, CV_8UC1, cv::Scalar::all(255)));

	cam->connect(0);
	cam->setParam(paramTypeCamera::paramInt::WIDTH, width);
	cam->setParam(paramTypeCamera::paramInt::HEIGHT, height);
	cam->setParam(paramTypeCamera::paramFloat::FPS, fps);
	cam->setParam(paramTypeCamera::paramFloat::GAIN, gain);
	cam->setParam(paramTypeIdpExpress::AcquisitionMode::EnableAcquisitionFrameRate);
	cam->parameter_all_print();

	cam->start();
	int th = 125;
	bool flag = true;
	std::thread thr([&] {
		while (flag) {
			//cam->captureFrame(img.data);
			cam->captureFrameStereo(img.data, img2.data);
		}
	});

	while (1) {
		cv::imshow("img", img);
		cv::imshow("img2", img2);
		cv::imshow("imgB", img > th);
		cv::imshow("img2B", img2 > th);
		int key = cv::waitKey(1);
		if (key == 'q')break;
		else if (key == '+') {
			th++;
			std::cout << th << std::endl;
		}
		else if (key == '-') {
			th--;
			std::cout << th << std::endl;
		}
	}
	flag = false;
	if (thr.joinable())thr.join();

	cam->stop();
	cam->disconnect();

	delete cam;
}