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
	vector<cv::Mat> Thresholds;
	vector<int> detect_cnt_log;
	cv::Mat dif;
	cv::Mat img;
	cv::Mat thrmask;
	cv::Mat difeval;
	cv::Mat thrmaskeval;
	cv::Mat laserthr,laserthr2;
	cv::Moments M;//形状変化した点群のモーメント
	cv::Rect roi(200, 170, 260, 200);
	int cnt = 0;
	int max_cnt = 10;


	char imgfolder[256] = "C:/Users/Mikihiro Ikura/Documents/GitHub/HighSpeedCamera/sample/ShapeMeasurement3D/results/190215/225621_success_detectlate/Pictures/frame";
	char diffolder[256] = "C:/Users/Mikihiro Ikura/Documents/GitHub/HighSpeedCamera/sample/ShapeMeasurement3D/results/190215/225621_success_detectlate/Pictures_diff/frame";
	char imgfile[256];
	char diffile[256];
	int start = 3500;
	int end = 6000;
	int diffframe = 100;

	for (int i = start; i < end; i++)
	{
		sprintf(imgfile, "%s%d.png", imgfolder, i);//取得画像読み取り
		img = cv::imread(imgfile,cv::IMREAD_GRAYSCALE);
		Imgs.push_back(img);
		sprintf(diffile, "%s%d.png", diffolder, i);//1フレーム差分画像読み取り
		dif = cv::imread(diffile, cv::IMREAD_GRAYSCALE);
		//cv::threshold(dif, thrmask, 12.0 , 255.0, cv::THRESH_BINARY);//1フレーム差分画像の閾値
		cv::imshow("img", img);//生画像
		//cv::imshow("threshold", thrmask);//差分画像＋閾値
		cv::waitKey(1);
		//Diffs.push_back(dif);
		//diffframe分の差分画像
		if (i>start+diffframe)
		{
			cv::Mat oldimg = Imgs[i - diffframe-start];
			cv::Mat newimg = Imgs[i-start];
			difeval = abs(newimg - oldimg);
			Diffs.push_back(difeval.clone());
			//レーザーが光っているところはThresholdかける
			cv::threshold(oldimg, laserthr, 50, 255, cv::THRESH_BINARY);
			cv::threshold(newimg, laserthr2, 50, 255, cv::THRESH_BINARY);//レーザーが光っているところはThresholdかける
			cv::GaussianBlur(laserthr, laserthr, cv::Size(17, 17), 0);
			cv::GaussianBlur(laserthr2, laserthr2, cv::Size(17, 17), 0);
			cv::threshold(laserthr, laserthr, 0, 255, cv::THRESH_BINARY);
			cv::threshold(laserthr2, laserthr2, 0, 255, cv::THRESH_BINARY);
			cv::bitwise_not(laserthr, laserthr);
			cv::bitwise_not(laserthr2, laserthr2);
			cv::bitwise_and(laserthr, laserthr2, laserthr);//二つのレーザー画像のThreshold画像をマージ
			int meanval = (int)mean(difeval)[0];//差分画像の平均画素値計算
			cv::threshold(difeval, thrmask, 15.0 + meanval, 255.0, cv::THRESH_BINARY);//15+画素値平均値の閾値超えを255に
			cv::bitwise_and(thrmask, laserthr, thrmask);//レーザーが映ったところは消去
			M = cv::moments(thrmask(roi));
			//cv::imshow("img", newimg);
			cv::imshow("detection", thrmask);
			cv::imshow("part_detect", thrmask(roi));
			printf("newimg No.%d :motion pix cnt: %d\n",i-start, (int)M.m00 / 255);
			Thresholds.push_back(thrmask(roi).clone());
			detect_cnt_log.push_back((int)M.m00 / 255);
			if ((int)M.m00 / 255 > 100) {
				cnt++;
				cv::waitKey(100);
				if (cnt > max_cnt) {
					cv::waitKey(5000);
				}
			}
			else {
				cnt = 0;
			}
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