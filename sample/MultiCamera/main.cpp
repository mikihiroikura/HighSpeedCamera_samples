
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
#define USE_BASLER
#define USE_XIMEA
#include <HSC/CameraFunc.hpp>

int main() {
	std::vector<Camera*> cams(3);
	cams[0] = getCamera(cameraType::IDPEXPRESS);
	cams[1] = getCamera(cameraType::BASLER);
	cams[2] = getCamera(cameraType::XIMEA);

	float  fps = 60.0f;
	float gain = 1.0f;

	std::vector<cv::Mat>imgs;
	imgs.emplace_back(cv::Mat(512, 512, CV_8UC1, cv::Scalar::all(255)));
	imgs.emplace_back(cv::Mat(480, 640, CV_8UC1, cv::Scalar::all(255)));
	imgs.emplace_back(cv::Mat(488, 648, CV_8UC1, cv::Scalar::all(255)));

	cams[0]->connect(0);
	cams[0]->setParam(paramTypeCamera::paramInt::WIDTH, 512);
	cams[0]->setParam(paramTypeCamera::paramInt::HEIGHT, 512);
	cams[0]->setParam(paramTypeCamera::paramFloat::FPS, fps);
	cams[0]->setParam(paramTypeCamera::paramFloat::GAIN, gain);
	cams[0]->setParam(paramTypeIdpExpress::AcquisitionMode::EnableAcquisitionFrameRate);
	cams[0]->parameter_all_print();

	cams[1]->connect(0);
	cams[1]->setParam(paramTypeCamera::paramInt::WIDTH, 640);
	cams[1]->setParam(paramTypeCamera::paramInt::HEIGHT, 480);
	cams[1]->setParam(paramTypeCamera::paramFloat::FPS, fps);
	cams[1]->setParam(paramTypeCamera::paramFloat::GAIN, gain);
	cams[1]->setParam(paramTypeBasler::Param::ExposureTime, 1950.0f);
	cams[1]->setParam(paramTypeBasler::AcquisitionMode::EnableAcquisitionFrameRate);
	//cams[1]->setParam(paramTypeBasler::FastMode::SensorReadoutModeFast);
	cams[1]->setParam(paramTypeBasler::GrabStrategy::LatestOnlyFrame);
	cams[1]->setParam(paramTypeBasler::CaptureType::MonocroGrab);
	cams[1]->parameter_all_print();

	cams[2]->connect(0);
	cams[2]->setParam(paramTypeCamera::paramInt::WIDTH, 648);
	cams[2]->setParam(paramTypeCamera::paramInt::HEIGHT, 488);
	cams[2]->setParam(paramTypeCamera::paramFloat::FPS, fps);
	cams[2]->setParam(paramTypeCamera::paramFloat::GAIN, gain);
	cams[2]->setParam(paramTypeXimea::AcquisitionMode::EnableAcquisitionFrameRate);
	cams[2]->setParam(paramTypeXimea::CaptureType::BayerGrab);

	for (auto cam : cams) {
		cam->start();
	}

	bool flag = true;
	std::thread thr([&] {
		while (flag) {
			for (auto i = 0; i < cams.size(); ++i)cams[i]->captureFrame(imgs[i].data);
		}
	});

	while (1) {
		for (auto i = 0; i < imgs.size(); ++i)cv::imshow("img : "+std::to_string(i), imgs[i]);
		int key = cv::waitKey(1);
		if (key == 'q')break;
	}
	flag = false;
	if (thr.joinable())thr.join();

	for (auto cam : cams) {
		cam->stop();
		cam->disconnect();
		delete cam;
	}

	return 0;
}