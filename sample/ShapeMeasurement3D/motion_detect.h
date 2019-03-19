#pragma once
#ifndef MOTION_DETECT_H_
#define MOTION_DETECT_H_

#include <vector>
#include <opencv2/opencv.hpp>
#include "RS232c.h"
#include <HSC/baslerClass.hpp>
#include <opencv2/core.hpp>

#include "parameters.h"

using namespace std;

//関数の宣言
extern void ShapeChangeDetectionMultiFrame(Capture *cap, bool *flg);

extern int outputlog_num_max;
extern cv::Mutex mutex2;
extern RS232c mirror;
extern bool numchange_flg;
extern vector<float> Xlaser_log;//レーザーの位置X座標ログ@pix座標系
extern cv::Mat thrmask;
extern bool shapedetection_flg;
extern cv::Moments M;//形状変化した点群のモーメント
extern vector<int> FrameNos;//newimgのフレーム番号
extern vector<int> Detect_num_log;//形状変化と検知した回数のログ
extern int detection_num;//形状変化と検出した回数
extern vector<int> Detect_pixcnt_log;//形状変化検出したピクセル点数のログ

#endif // MOTION_DETECT_H_

