#define _CRT_SECURE_NO_WARNINGS
#include <thread>
#include <opencv2/opencv.hpp>
#include <conio.h>


//#define USE_EOSENS
//#define USE_XIMEA
#define USE_IDPEXPRESS
//#define USE_BASLER
#include <HSC/CameraUI.hpp>


#ifdef _DEBUG
#define CAMERA_EXT "d.lib"
#else
#define CAMERA_EXT ".lib"
#endif

#define STR_EXP(__A) #__A
#define STR(__A) STR_EXP(__A)
#define CV_VER STR(CV_VERSION_MAJOR) STR(CV_VERSION_MINOR) STR(CV_VERSION_REVISION)
#pragma comment(lib, "opencv_world" CV_VER CAMERA_EXT)

int main() {

#ifdef USE_IDPEXPRESS
	CameraUI cui(CameraUI::cameraType::IDPEXPRESS);
	int width = 512;
	int height = 512;
#elif defined(USE_XIMEA)
	CameraUI cui(CameraUI::cameraType::XIMEA);
	int width = 648;
	int height = 488;
#elif defined(USE_EOSENS)
	CameraUI cui(CameraUI::cameraType::EOSENS);
	int width = 1280;
	int height = 1024;
#elif defined(USE_BASLER)
	CameraUI cui(CameraUI::cameraType::BASLER);
	int width = 640;
	int height = 480;
#endif
	float  fps = 250.0f;
	float gain = 12.0f;

	cui.connect(0);
	cui.setParam(paramTypeCamera::paramInt::WIDTH, width);
	cui.setParam(paramTypeCamera::paramInt::HEIGHT, height);
	cui.setParam(paramTypeCamera::paramFloat::FPS, fps);
	cui.setParam(paramTypeCamera::paramFloat::GAIN, gain);
#ifdef USE_IDPEXPRESS
	cui.setParam(paramTypeIdpExpress::AcquisitionMode::EnableAcquisitionFrameRate);
#elif defined(USE_XIMEA)
	cui.setParam(paramTypeXimea::AcquisitionMode::EnableAcquisitionFrameRate);
	cui.setParam(paramTypeXimea::CaptureType::MonocroGrab);
#elif defined(USE_EOSENS)

#elif defined(USE_BASLER)
/* setParam関連は必ずスタート前に終わらせる */
	cui.setParam(paramTypeBasler::Param::ExposureTime, 1950.0f);
	cui.setParam(paramTypeBasler::AcquisitionMode::EnableAcquisitionFrameRate);
	cui.setParam(paramTypeBasler::FastMode::SensorReadoutModeFast);
	cui.setParam(paramTypeBasler::GrabStrategy::OneByOne);
	cui.setParam(paramTypeBasler::CaptureType::MonocroGrab);
	cui.parameter_all_print();
#endif
	cui.start();
//	cui2.start();
	width = cui.getParam(paramTypeCamera::paramInt::WIDTH);
	height = cui.getParam(paramTypeCamera::paramInt::HEIGHT);
	std::cout << "width: " << width << " ,  height: " << height << std::endl;
	std::vector<cv::Mat> imgs, imgCs;
	imgs.emplace_back(cv::Mat(height, width, CV_8UC1, cv::Scalar::all(255)));
	imgs.emplace_back(cv::Mat(height, width, CV_8UC1, cv::Scalar::all(255)));
//	imgs.emplace_back(cv::Mat(height, width, CV_8UC1, cv::Scalar::all(255)));

	for(int i=0;i<imgs.size();++i)imgCs.emplace_back(cv::Mat(imgs[i].size(), CV_8UC3, cv::Scalar::all(255)));
	int imgBufSize = 100;
	std::vector<std::vector<cv::Mat>> imgCsBuf;
	for (int i = 0; i<imgs.size(); ++i) imgCsBuf.emplace_back(std::vector<cv::Mat>(imgBufSize, cv::Mat(imgs[i].size(), CV_8UC3, cv::Scalar::all(255))));
	bool flag = true;
	bool logFlag = false;
	int index = 0;

	std::thread thr([&] {
		while (flag) {

#ifdef USE_IDPEXPRESS
			cui.captureFrameStereo((imgs[0].data), (imgs[1].data));
#else
			cui.captureFrame((imgs[0].data));
#endif
			if (logFlag) {
				for (int i = 0; i < imgs.size(); ++i)imgCsBuf[i][index] = imgs[i].clone();
				/*for (int i = 0; i < imgs.size(); ++i) {
					cv::cvtColor(imgs[i], imgCs[i], cv::COLOR_BayerGB2BGR);
					imgCsBuf[i][index] = imgCs[i].clone();
				}*/
				cv::Mat test;
				cv::cvtColor(imgs[0], test, cv::COLOR_BayerGB2GRAY);
				std::stringstream ss;
				ss << "../picture/image" << std::setfill('0') << std::setw(3) << index << ".png";
				cv::imwrite(ss.str(), test);
				index++;
				logFlag = false;
				if (index >= imgBufSize) {
					flag = false;
				}
			}
		}
	});
	int key = 0;
	while (1) {
		for (int i = 0; i < imgs.size(); ++i)cv::imshow("test " + std::to_string(i), imgs[i]);
		key = cv::waitKey(1);
		if (key == 'q')break;
		else if (key == 's') logFlag = true;
	}
	logFlag = true;
	//flag = false;
	if (thr.joinable())thr.join();
	cui.stop();
	cui.disconnect();

	/// 録画データ確認
	index = 0;
	while (1) {
		for (int i = 0; i < imgs.size(); ++i)cv::imshow("test " + std::to_string(i), imgCsBuf[i][index]);
		index = (index + 1) % imgBufSize;
		if (cv::waitKey(1) == 'q')break;
	}


	/// 一枚ずつ確認用
	index = 0;
	while (1) {
		for (int i = 0; i < imgs.size(); ++i)cv::imshow("test " + std::to_string(i), imgCsBuf[i][index]);

		int key = cv::waitKey(1);
		if (key == 'q')break;
		else if (key == '+') {
			index = (index + 1) % imgBufSize;
			std::cout << index << std::endl;
		}
		else if (key == '-') {
			index = (index + imgBufSize - 1) % imgBufSize;
			std::cout << index << std::endl;
		}
	}

	/// 画像の保存
	std::cout << "Save Start." << std::endl;
	for (int i = 0; i < imgBufSize; ++i) {
		for (int j = 0; j < imgs.size(); ++j) {
			std::stringstream ss;
			ss << "../picture/image" << j << "_" << std::setfill('0') << std::setw(3) << i << ".png";
			cv::imwrite(ss.str(), imgCsBuf[j][i]);
		}
	}
	std::cout << "Save End." << std::endl;


	/// 1枚の画像に合成
	int totalWidth = 0, maxHeight = 0;
	for (auto i = 0; i < imgs.size(); ++i) {
		totalWidth += imgs[i].size().width;
		maxHeight = std::max<int>(maxHeight,imgs[i].size().height);
	}
	for (int i = 0; i < imgBufSize; ++i) {
		cv::Mat connectImg(maxHeight, totalWidth, imgCsBuf[0][i].depth());
		cv::Rect roi(0,0,0,0);
		for (int j = 0; j < imgs.size(); ++j) {
			roi.width = imgCsBuf[j][i].size().width;
			roi.height = imgCsBuf[j][i].size().height;
			(imgCsBuf[j][i]).copyTo(connectImg(roi));
			cv::rectangle(connectImg, roi, cv::Scalar(255), 1);
			roi.x += imgCsBuf[j][i].size().width;
		}
		std::stringstream ss;
		ss << "../picture/connect" << std::setfill('0') << std::setw(3) << i << ".png";
		cv::imwrite(ss.str(), connectImg);
	}
	


	return 0;
}