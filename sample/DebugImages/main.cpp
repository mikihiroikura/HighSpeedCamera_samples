#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <thread>
#include <string>
#include <vector>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <direct.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <Windows.h>

#ifdef _DEBUG
#define CAMERA_EXT "d.lib"
#else
#define CAMERA_EXT ".lib"
#endif

#define STR_EXP(__A) #__A
#define STR(__A) STR_EXP(__A)
#define CV_VER STR(CV_VERSION_MAJOR) STR(CV_VERSION_MINOR) STR(CV_VERSION_REVISION)
#pragma comment(lib, "opencv_world" CV_VER CAMERA_EXT)

using namespace std;

#pragma warning(disable:4996)

int main() {
	vector<cv::Mat> Diffs;
	vector<cv::Mat> Imgs;
	cv::Mat dif;
	cv::Mat img;
	cv::Mat thrmask;
	cv::Mat difeval;
	cv::Mat thrmaskeval;
	cv::Mat laserthr,laserthr2;
	cv::Moments M;//形状変化した点群のモーメント
	cv::Rect roi(150, 150, 340, 210);


	char imgfolder[256] = "C:/Users/Mikihiro Ikura/Documents/GitHub/HighSpeedCamera/sample/ShapeMeasurement3D/results/190207/192932/Pictures/frame";
	char diffolder[256] = "C:/Users/Mikihiro Ikura/Documents/GitHub/HighSpeedCamera/sample/ShapeMeasurement3D/results/190207/192932/Pictures_diff/frame";
	char imgfile[256];
	char diffile[256];
	int start = 4000;
	int end = 10000;
	int diffframe = 30;

	for (int i = start; i < end; i++)
	{
		sprintf(imgfile, "%s%d.png", imgfolder, i);//取得画像読み取り
		img = cv::imread(imgfile,cv::IMREAD_GRAYSCALE);
		Imgs.push_back(img);
		sprintf(diffile, "%s%d.png", diffolder, i);//1フレーム差分画像読み取り
		dif = cv::imread(diffile, cv::IMREAD_GRAYSCALE);
		cv::threshold(dif, thrmask, 12.0 , 255.0, cv::THRESH_BINARY);//1フレーム差分画像の閾値
		cv::imshow("img", img);//生画像
		cv::imshow("threshold", thrmask);//差分画像＋閾値
		cv::waitKey(1);
		Diffs.push_back(dif);
		//diffframe分の差分画像
		if (i>start+diffframe)
		{
			cv::Mat oldimg = Imgs[i - diffframe-start];
			cv::Mat newimg = Imgs[i-start];
			difeval = abs(newimg - oldimg);
			//レーザーが光っているところはThresholdかける
			cv::threshold(oldimg, laserthr, 180, 255,cv::THRESH_BINARY_INV);
			cv::threshold(newimg, laserthr2, 180, 255,cv::THRESH_BINARY_INV);
			cv::bitwise_and(laserthr, laserthr2, laserthr);//二つのレーザー画像のThreshold画像をマージ
			int meanval = (int)mean(difeval)[0];//30フレーム差分画像の平均をThresholdに加える
			cv::threshold(difeval, thrmaskeval, 12.0 + meanval, 255.0, cv::THRESH_BINARY);
			cv::bitwise_and(thrmaskeval, laserthr, thrmaskeval);//レーザーが映ったところは消去
			M = cv::moments(thrmaskeval(roi));
			//cv::imshow("img", newimg);
			cv::imshow("detection", thrmaskeval);
			cv::imshow("part_detect", thrmaskeval(roi));
			printf("motion pix cnt: %d\n", (int)M.m00 / 255);
		}
	}
	printf("saveimg\n");
	//ここで移動体位置検出を試す
	/*for (int i = diffframe; i < Imgs.size(); i++)
	{
		cv::Mat oldimg = Imgs[i-diffframe];
		cv::Mat newimg = Imgs[i];
		dif = abs(newimg - oldimg);
		int meanval = (int)mean(dif)[0];
		cv::threshold(dif, thrmask, 12.0 + meanval, 255.0, cv::THRESH_BINARY);
		cv::imshow("img", newimg);
		cv::imshow("threshold", thrmask);
		cv::waitKey(1);
	}*/


	return 0;
}