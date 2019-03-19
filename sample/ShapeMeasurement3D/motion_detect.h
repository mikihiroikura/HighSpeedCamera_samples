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

//�֐��̐錾
extern void ShapeChangeDetectionMultiFrame(Capture *cap, bool *flg);

extern int outputlog_num_max;
extern cv::Mutex mutex2;
extern RS232c mirror;
extern bool numchange_flg;
extern vector<float> Xlaser_log;//���[�U�[�̈ʒuX���W���O@pix���W�n
extern cv::Mat thrmask;
extern bool shapedetection_flg;
extern cv::Moments M;//�`��ω������_�Q�̃��[�����g
extern vector<int> FrameNos;//newimg�̃t���[���ԍ�
extern vector<int> Detect_num_log;//�`��ω��ƌ��m�����񐔂̃��O
extern int detection_num;//�`��ω��ƌ��o������
extern vector<int> Detect_pixcnt_log;//�`��ω����o�����s�N�Z���_���̃��O

#endif // MOTION_DETECT_H_

