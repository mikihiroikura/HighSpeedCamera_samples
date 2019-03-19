#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <thread>
#include <string>
#include <vector>
#include "RS232c.h"
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <direct.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <Windows.h>
#define _USE_MATH_DEFINES
#include <math.h>
#define GLFW_NO_GLU

#ifdef _DEBUG
#define CAMERA_EXT "d.lib"
#else
#define CAMERA_EXT ".lib"
#endif

//opencv_world341(d).lib�̒ǉ�
#define STR_EXP(__A) #__A
#define STR(__A) STR_EXP(__A)
#define CV_VER STR(CV_VERSION_MAJOR) STR(CV_VERSION_MINOR) STR(CV_VERSION_REVISION)
#pragma comment(lib, "opencv_world" CV_VER CAMERA_EXT)

#define MAX_

#include <HSC/baslerClass.hpp>
#pragma comment(lib, "BaslerLib" CAMERA_EXT)

#pragma warning(disable:4996)

#include "motion_detect.h"
#include "graphics.h"
#include "parameters.h"


//DEFINE�Q
#define ROBOT_MOVE_//���{�b�g�𓮂���
#define SAVE_IMG_         //�摜�Ɠ����Log�Ɏc��
//#define MATLAB_GRAPHICS_  //MATLAB���N�����CLog���v���b�g����

using namespace std;

//�O���[�o���ϐ�
double result, logtime, det;
double moment, mass, cog, rowmass, colmass;
int rowcnt;
cv::Mat lhand, rhand, sol;
float timeout = 30.0;
cv::Mat campnt, wldpnt;
double hw;
int outputlog_num_max = 40;//�o�͂���3�������W�̐�
int outputlog_num_part = 20;//�����X�L��������Ƃ��̃��[�U�[�{��
bool numchange_flg = false;//�����X�L�����ɂȂ����Ƃ��ɗ��Ă�t���O
int numchange_timing = -1;//�����X�L�����ɂȂ�������Vector�z���Index�ԍ�
int imgsavecnt = 0;//�摜�ۑ��ꏊ
int outputimgcnt = 0;//�W���o�͂̉摜��ꏊ
RS232c mirror;
vector<float> Xlaser_log(outputlog_num_max * 2, 0);//���[�U�[�̈ʒuX���W���O@pix���W�n
cv::Mat thrmask = cv::Mat(480, 640, CV_8UC1, cv::Scalar::all(255));
bool shapedetection_flg = false;
cv::Moments M;//�`��ω������_�Q�̃��[�����g
vector<int> FrameNos;//newimg�̃t���[���ԍ�
vector<int> Detect_num_log;//�`��ω��ƌ��m�����񐔂̃��O
int detection_num = 0;//�`��ω��ƌ��o������
vector<int> Detect_pixcnt_log;//�`��ω����o�����s�N�Z���_���̃��O

//���ؒf�@�͈͎w��
int LS_sta_row = 150;
int LS_end_row = 300;

//bar�ɉf�郌�[�U�[���ʌv�Z�͈͎w��
int PL_sta_row = 390;
int PL_end_row = 460;
cv::Rect bar_roi(60, 390, 300, 70);


//Thread�r������
cv::Mutex mutex,mutex2,flg;

//�����ϐ�
LARGE_INTEGER freq, start,logend;

//�v���g�^�C�v�錾
void TakePicture(Capture *cap, bool *flg);
int CalcHeights(Capture *cap);
void OutPutLogs(Capture *cap, bool *flg);
void AxisToPoint(RS232c *axis, int *pointnum);//�P�����{�b�g�̃|�C���g�w��ړ�

int main(int argc, char *argv[]) {
	LARGE_INTEGER end;
	if (!QueryPerformanceFrequency(&freq)) { return 0; }// �P�ʏK��
	//�J�����p�����[�^�[
	int width = 640;
	int height = 480;
	float  fps = 750.0f;
	float gain = 1.0f;

	#ifdef ROBOT_MOVE_
	// �P�����{�b�g��RS232�ڑ�
	RS232c axis;
	char buf[256];
	axis.Connect("COM3", 38400, 8,ODDPARITY, 0, 0, 0, 20000, 20000);
	//�P�����{�b�g�T�[�{����
	axis.Send("@SRVO1.1\r\n");
	axis.Read_CRLF(buf, 256);
	printf(buf);
	Sleep(1000);
	//�X�^�[�g�n�_�ֈړ�
	int startp = 30;
	AxisToPoint(&axis, &startp);
	#endif // ROBOT_MOVE_


	//�L���v�`���p�̍\���̂̐錾
	Capture cap;
	cap.in_img = (cv::Mat(height, width, CV_8UC1, cv::Scalar::all(255)));
	cap.flag = true;
	cap.pic_cnt = 0;
	cap.Processed.push_back(false);//���False��ǉ����Ă���
	cap.cam.connect(0);
	cap.cam.setParam(paramTypeCamera::paramInt::WIDTH, width);
	cap.cam.setParam(paramTypeCamera::paramInt::HEIGHT, height);
	cap.CoGs.resize(height, 0);
	cap.cam.setParam(paramTypeBasler::FastMode::SensorReadoutModeFast);
	cap.cam.setParam(paramTypeBasler::Param::ExposureTime, 1280.0f);
	cap.cam.setParam(paramTypeCamera::paramFloat::FPS, fps);
	cap.cam.setParam(paramTypeCamera::paramFloat::GAIN, gain);
	//cap.cam.setParam(paramTypeBasler::AcquisitionMode::EnableAcquisitionFrameRate);//�t���[�����[�g���Œ肵�ăp�����[�^���߂�
	cap.cam.setParam(paramTypeBasler::AcquisitionMode::TriggerMode);//MBED����Trigger���������Ƃ��ɎB������
	cap.cam.setParam(paramTypeBasler::GrabStrategy::LatestOnlyFrame);//OnebyOne:�B���摜���̂ĂȂ��Œx��Ă��o�b�t�@�ɒ��߂�1�������� LatestOnlyFrame:��Ƀo�b�t�@���X�V���čŐV�̉摜���B�葱����
																	 /*cap.cam.setParam(paramTypeBasler::CaptureType::BayerBGGrab);
																	 cap.cam.setParam(paramTypeBasler::CaptureType::ColorBGRGrab);*/
	cap.cam.setParam(paramTypeBasler::CaptureType::MonocroGrab);
	cap.cam.parameter_all_print();
	//Calibration���ʂ̊i�[
	double a = 0, b = 0, c = 0, d = 0,h = 0;
	double R[9];
	FILE* fp;
	fp = fopen("cameraplanebarparamsinterpolate.csv", "r");
	fscanf(fp, "%lf,%lf,%lf,%lf,", &a, &b, &c, &d);
	cap.map_coefficient.push_back(a);
	cap.map_coefficient.push_back(b);
	cap.map_coefficient.push_back(c);
	cap.map_coefficient.push_back(d);
	fscanf(fp, "%lf,%lf,%lf,%lf,", &a, &b, &c, &d);//�s��̏o�͂���鏇�Ԃɒ���
	cap.stretch_mat.push_back(a);
	cap.stretch_mat.push_back(c);
	cap.stretch_mat.push_back(b);
	cap.stretch_mat.push_back(d);
	fscanf(fp, "%lf,%lf,", &a, &b);
	cap.distortion.push_back(a);
	cap.distortion.push_back(b);
	fscanf(fp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,", &R[0], &R[1], &R[2], &R[3], &R[4], &R[5], &R[6], &R[7], &R[8]);
	cv::Mat Rs(cv::Size(3, 3), CV_64F, R);
	cap.Rots.push_back(Rs.clone());
	//�_��̋P�_��X���W�Q
	for (size_t i = 0; i < 100; i++){
		fscanf(fp, "%lf,", &cap.barX[i]);
	}
	//���ʂ̎�ax+by+cz=1�̃p�����[�^
	for (size_t i = 0; i < 100; i++) {
		fscanf(fp, "%lf,", &cap.planeA[i]);
	}
	for (size_t i = 0; i < 100; i++) {
		fscanf(fp, "%lf,", &cap.planeB[i]);
	}
	for (size_t i = 0; i < 100; i++) {
		fscanf(fp, "%lf,", &cap.planeC[i]);
	}
	fclose(fp);

	//��ɉ摜Vector��p�ӂ���
	for (size_t i = 0; i < (int)(timeout*fps+100); i++)
	{
		cap.Pictures.push_back((cv::Mat(480, 640, CV_8UC1, cv::Scalar::all(255))));
		cap.Processed.push_back(false);
	}
	

	//det�̌v�Z������
	det = 1.0 / (cap.stretch_mat[0] - cap.stretch_mat[1] * cap.stretch_mat[2]);//�O�ł��

	//�J�����N��
	cap.cam.start();

	//�~���[����̂��߂�MBED�}�C�R���ւ�RS232�ڑ�
	mirror.Connect("COM4", 115200, 8, NOPARITY,0,0,0,5000,20000);

	//�X���b�h�쐬
	bool flag = true;
	thread thr(TakePicture, &cap, &flag);//�\���̂ɉ摜��ۑ�����Thread
	thread thr2(OutPutLogs, &cap, &flag);//���݂̉摜�C�v�Z���x���f�o�b�N�o�͂���Thread
	thread thr3(writepointcloud, &cap,&flag);//OpenGL�Ōv�Z�����_�Q���o�͂���Thread
	thread thr4(ShapeChangeDetectionMultiFrame, &cap, &flag);//�����t���[�������摜����ړ��̌��m
#ifdef ROBOT_MOVE_
	int endp = 31;//����w��ԍ�
	thread thr5(AxisToPoint, &axis, &endp);//���{�b�g��350mm�։���
#endif // ROBOT_MOVE_

	
	
	Sleep(1);//thread�̂�1ms���s���C�摜���i�[������
	if (!QueryPerformanceCounter(&start)) { return 0; }

	//���C�����[�v
	while (flag)
	{
		if (cap.Processed[cap.pic_cnt]){//�ʐ^������Ƃ���
			CalcHeights(&cap);//���x�v�Z
			if (!QueryPerformanceCounter(&end)) { return 0; }
			logtime = (double)(end.QuadPart - start.QuadPart) / freq.QuadPart;
			cap.Log_times.push_back(logtime);
			if (numchange_flg){//�t���O����������
				numchange_timing = cap.Log_times.size() - 1;
				numchange_flg = false;
			}
			
		}
		
	}
	mirror.Send("q");
	if (thr.joinable())thr.join();
	if (thr2.joinable())thr2.join();
	if (thr3.joinable())thr3.join();
	if (thr4.joinable())thr4.join();
#ifdef ROBOT_MOVE_
	if (thr5.joinable())thr5.join();
#endif // ROBOT_MOVE_

	
	cap.cam.stop();
	cap.cam.disconnect();

	//���O�ۑ�
	//���ʕۑ��p�t�@�C��������t���ō쐬���J��
	FILE* fr;
	FILE* fr2;
	time_t timer;
	struct tm now;
	struct tm *local;
	timer = time(NULL);
	localtime_s(&now, &timer);
	char dir0[256];
	strftime(dir0, 256, "C:/Users/Mikihiro Ikura/Documents/GitHub/HighSpeedCamera/sample/ShapeMeasurement3D/results/%y%m%d", &now);
	_mkdir(dir0);
	char dir[256];
	strftime(dir, 256, "C:/Users/Mikihiro Ikura/Documents/GitHub/HighSpeedCamera/sample/ShapeMeasurement3D/results/%y%m%d/%H%M%S", &now);
	_mkdir(dir);
	char dir2[256];
	strftime(dir2, 256, "C:/Users/Mikihiro Ikura/Documents/GitHub/HighSpeedCamera/sample/ShapeMeasurement3D/results/%y%m%d/%H%M%S/Pictures", &now);
	_mkdir(dir2);
	strftime(dir2, 256, "C:/Users/Mikihiro Ikura/Documents/GitHub/HighSpeedCamera/sample/ShapeMeasurement3D/results/%y%m%d/%H%M%S/Pictures_diff", &now);
	_mkdir(dir2);
	char filename[256];
	char ShapeChange[256];
	strftime(filename, 256, "results/%y%m%d/%H%M%S/LS_result.csv", &now);
	strftime(ShapeChange, 256, "results/%y%m%d/%H%M%S/ShapeChange_result.csv", &now);
	fr = fopen(filename, "w");
	fr2 = fopen(ShapeChange, "w");

	//�P�����{�b�g�ؒf
#ifdef ROBOT_MOVE_
	axis.Send("@SRVO0.1\r\n");
	axis.Read_CRLF(buf, 256);
#endif // ROBOT_MOVE_

	
	//cap����Log��ۑ�
	printf("saving Logs...   ");
	for (int i = 0; i < cap.Log_times.size(); i++){
		fprintf(fr, "%lf,", cap.Log_times[i]);
		/*for (int j = 0; j < cap.Row_num_Logs[i].size(); j++){
			fprintf(fr, "%d,", cap.Row_num_Logs[i][j]);
		}*/
		//fprintf(fr, "\n");
		if (i == numchange_timing) { 
			outputlog_num_max = outputlog_num_part;
			fprintf(fr, "%d,",1);
		}
		#ifdef ROBOT_MOVE_
		if (i==0){
			fprintf(fr, "%d,", 2);
		}
		#endif // ROBOT_MOVE_

		fprintf(fr, "\n");
		for (int j = 0; j < cap.Worlds_Logs[i].size(); j++) {
			fprintf(fr, "%lf,", cap.Worlds_Logs[i][j].at<double>(0, 0));
		}
		if (cap.Worlds_Logs[i].size()==0){
			fprintf(fr, "%lf,", 0);
		}
		
		fprintf(fr, "\n");
		for (int j = 0; j < cap.Worlds_Logs[i].size(); j++) {
			fprintf(fr, "%lf,", cap.Worlds_Logs[i][j].at<double>(0, 1));
		}
		if (cap.Worlds_Logs[i].size() == 0) {
			fprintf(fr, "%lf,", 0);
		}
		fprintf(fr, "\n");
		for (int j = 0; j < cap.Worlds_Logs[i].size(); j++) {
			fprintf(fr, "%lf,", cap.Worlds_Logs[i][j].at<double>(0, 2));
		}
		if (cap.Worlds_Logs[i].size() == 0) {
			fprintf(fr, "%lf,",0);
		}
		fprintf(fr, "\n");
	}
	for (int i = 0; i < FrameNos.size(); i++)//�����摜�ۑ�
	{
		fprintf(fr2, "%d,%d,%d\n", FrameNos[i],Detect_pixcnt_log[i],Detect_num_log[i]);
	}
	printf("Logs finish!\n");
	fclose(fr);
	fclose(fr2);

	//�摜��ۑ�
	#ifdef SAVE_IMG_
	printf("saving Imgs...   ");
	char videoname[256];
	char picturename[256];
	char picsubname[128];
	char diffname[256];
	char diffsubname[128];
	strftime(videoname, 256, "results/%y%m%d/%H%M%S/video.avi", &now);
	strftime(picsubname, 128, "results/%y%m%d/%H%M%S/Pictures/frame", &now);
	strftime(diffsubname, 128, "results/%y%m%d/%H%M%S/Pictures_diff/frame", &now);
	//cv::VideoWriter writer(videoname, cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 750.0, cv::Size(640, 480), false);//���m�N���Ȃ̂�False�w��
	//if (!writer.isOpened()) { return -1; }
	for (int i = 0; i < imgsavecnt; i++) {//���摜�ۑ�
		//writer << cap.Pictures[i].clone();
		sprintf(picturename, "%s%d.png", picsubname, i);//png�t���k
		cv::imwrite(picturename, cap.Pictures[i]);
	}
	printf("Imgs finish!\n");
	//writer.release();
	#endif // SAVE_IMG_

	//MATLAB��Log�̏o�͂�����
	#ifdef MATLAB_GRAPHICS_
	printf("MATLAB Open...   ");
	int ret = 0;
	char matdir[256] = "C:\\Users\\Mikihiro Ikura\\Documents\\GitHub\\ShowLSLogs\\LS_result.csv";
	int cpy = CopyFileA(filename, matdir, FALSE);
	if (cpy) { printf("Copy success!   "); }
	else { printf("Copy failed...   "); }
	char matlabcmd[256] = "matlab -sd \"C:\\Users\\Mikihiro Ikura\\Documents\\GitHub\\ShowLSLogs\" -r ShowLSLogs -nosplash";
	ret = system(matlabcmd);
	if (ret == 0) { printf("MATLAB activate! \n"); }
	else{printf("error! \n");}
	#endif // MATLAB_GRAPHICS_

	printf("ALL FINALIZE ! \n");
	system("pause");
	return 0;
}

//thread���̊֐�
//�����Ccap�Ɏʐ^���i�[����
void TakePicture(Capture *cap, bool *flg) {
	cv::Mat temp = cv::Mat(480, 640, CV_8UC1, cv::Scalar::all(255));
	while (*flg) {
		//�����͔r�����䂵�Ȃ�
		cap->cam.captureFrame(temp.data);
		{
			cv::AutoLock lock(mutex);
			cap->Pictures[imgsavecnt] = temp.clone();
		}
		cap->Processed[imgsavecnt] = true;//���ǉ����ꂽ�ʐ^�̔ԍ�������True�ɂ���
		outputimgcnt = imgsavecnt;
		imgsavecnt++;
	}
}


//thread2���̊֐�
//�W���o�͂Ō��݂̎ʐ^�C�����ƍ��x
void OutPutLogs(Capture *cap ,bool *flg) {
	while (1)
	{
		if (thrmask.data!=NULL&&outputimgcnt>3) {
			cv::imshow("img", cap->Pictures[outputimgcnt - 3]);
			cv::imshow("Diffs", thrmask);
		}
		int key = cv::waitKey(1);
		if (key == 'q') {
			*flg = false;
			break;
		}
		else if (key == 'd') {//�~���[�̃��[�h�ύX�f�o�b�N�p
			mirror.Send("d");
		}
		if (logtime > timeout) { 
			*flg = false;
			break;
		}
		if (cap->Worlds_Logs.size()>0){
			if (cap->Worlds_Logs[cap->Worlds_Logs.size() - 1].size() == 0) { printf("Time: ,%lf, s Bar: ,%lf, mm  Height: ,NONE    , mm  Detectcnt: %d, Continuous Detect: %d\n", logtime,cap->barX[cap->barXint], (int)M.m00 / 255,detection_num); }
			else {
				result = cap->Worlds_Logs[cap->Worlds_Logs.size() - 1][0].at<double>(0, 2);
				printf("Time: ,%lf, s Bar: ,%lf, mm  Height: ,%.4lf, mm Detectcnt: %d, Continuous Detect: %d\n", logtime, cap->barX[cap->barXint],result, (int)M.m00 / 255,detection_num);
			}
		}
		
	}
}

//main loop���ł̍��x�v�Z
int CalcHeights(Capture *cap) {
	//�O��g�p�����z���Clear
	cap->CoGs.clear();
	cap->IdealPixs.clear();
	cap->Worlds.clear();
	//cap->row_num.clear();
	cap->barPoint.clear();
	cap->barIdealPixs.clear();
	shapedetection_flg = false;//�r������CPictures����Clone������A�����b�N
	//�摜�z��Q���玟�̏����J�E���^�[�̔ԍ��̉摜��in_img�Ɋi�[����
	{
		cv::AutoLock lock(mutex);
		cap->in_img = cap->Pictures[cap->pic_cnt].clone();
	}
	cap->pic_cnt++;	
	shapedetection_flg = true;//�A�����b�N
	if (cap->in_img.data != NULL) {
		//�P�x�d�S�̌v�Z
		cv::threshold(cap->in_img, cap->out_img, 150.0, 255.0, cv::THRESH_BINARY);
		cv::bitwise_and(cap->in_img, cap->out_img, cap->out_img);
		//Bar�ɉf�郌�[�U�[�P�x�d�S�̌v�Z
		cv::threshold(cap->in_img(bar_roi), cap->bar_img, 70, 255.0, cv::THRESH_BINARY);
		cv::bitwise_and(cap->in_img(bar_roi), cap->bar_img, cap->bar_img);
		//Reference�_�ɉf��_�Q�̋P�x�d�S���Ƃ�
		for (int i = 0; i < cap->bar_img.rows; i++) {
			moment = 0.0;
			mass = 0.0;
			for (int j = 0; j < cap->bar_img.cols; j++) {
				mass += cap->bar_img.data[i*cap->bar_img.cols + j];
				moment += j * cap->bar_img.data[i*cap->bar_img.cols + j];
			}
			if (mass <= 0) { cog = 0; }
			else {
				cog = moment / mass + bar_roi.x;
			}
			cap->barPoint.push_back(cog);
		}
		//LINE�̕����̋P�x�d�S���l������
		for (int i = LS_sta_row; i < LS_end_row; i++) {
			moment = 0.0;
			mass = 0.0;
			for (int j = 0; j < cap->out_img.cols; j++) {
				mass += cap->out_img.data[i*cap->out_img.cols + j];
				moment += j * cap->out_img.data[i*cap->out_img.cols + j];
			}
			if (mass > 0) {
				cog = moment / mass;
			}
			cap->CoGs.push_back(cog);
		}
		{
			cv::AutoLock coglock(mutex2);//Xlaser_log�̔r������
			if (cap->CoGs.size() == 0) { Xlaser_log.push_back(0); }
			else { Xlaser_log.push_back((float)cap->CoGs[cap->CoGs.size() / 2]); }
			Xlaser_log.erase(Xlaser_log.begin());
		}//�A�����b�N
		cap->CoGs_Logs.push_back(cap->CoGs);//�P�x�d�S�̃��O�ۑ�
		//�P�x�d�S�˗��z�s�N�Z�����W�ɕϊ�
		vector<double> idpixs;
		double u, v, w;//����for���ł��g��
		double ud, vd;
		double totalcog = 0;
		double cogcnt = 0;
		double barcog;
		//�_�ɉf��P�_���S��X���W@pixel���W����C�⊮�����_�Q��p���ĕ��ʃp�����[�^���擾
		for (int i = 0; i < cap->barPoint.size(); i++)
		{
			if (cap->barPoint[i] == 0) { continue; }
			totalcog += cap->barPoint[i];
			cogcnt++;
		}
		barcog = totalcog / cogcnt;
		//�������[�U�[���ʂ���ԗ̓��Ȃ���s
		if (cap->barX[0]<barcog && barcog<cap->barX[99]) {
			cap->barXint = 0;
			for (int i = 0; i < 100; i++)
			{
				if (cap->barX[i]<barcog&&cap->barX[i + 1]>barcog) {
					cap->barXint = i;
					break;
				}
			}
			cap->laser_plane[0] = (cap->planeA[cap->barXint] * (cap->barX[cap->barXint + 1] - barcog) + cap->planeA[cap->barXint + 1] * (barcog - cap->barX[cap->barXint])) / (cap->barX[cap->barXint + 1] - cap->barX[cap->barXint]);
			cap->laser_plane[1] = (cap->planeB[cap->barXint] * (cap->barX[cap->barXint + 1] - barcog) + cap->planeB[cap->barXint + 1] * (barcog - cap->barX[cap->barXint])) / (cap->barX[cap->barXint + 1] - cap->barX[cap->barXint]);
			cap->laser_plane[2] = (cap->planeC[cap->barXint] * (cap->barX[cap->barXint + 1] - barcog) + cap->planeC[cap->barXint + 1] * (barcog - cap->barX[cap->barXint])) / (cap->barX[cap->barXint + 1] - cap->barX[cap->barXint]);

			//�n�ʂɂ����郌�[�U�[�̗��z�s�N�Z�����W�̌v�Z
			for (int i = 0; i < cap->CoGs.size(); i++)
			{
				if (cap->CoGs[i] == 0) { continue; }
				ud = cap->CoGs[i] - cap->distortion[0];
				vd = (double)(i + LS_sta_row) - cap->distortion[1];
				u = det * (ud - cap->stretch_mat[1] * vd);
				v = det * (-cap->stretch_mat[2] * ud + cap->stretch_mat[0] * vd);
				idpixs.push_back(u);
				idpixs.push_back(v);
				cap->IdealPixs.push_back(idpixs);
				idpixs.clear();
			}

			//���z�s�N�Z�����W�˒����̎��ƃ��[�U�[���ʂ̋���
			double phi, lambda, h;
			for (size_t i = 0; i < cap->IdealPixs.size(); i++)
			{
				u = cap->IdealPixs[i][0];
				v = cap->IdealPixs[i][1];
				phi = hypot(u, v);
				w = cap->map_coefficient[0] + cap->map_coefficient[1] * pow(phi, 2)
					+ cap->map_coefficient[2] * pow(phi, 3) + cap->map_coefficient[3] * pow(phi, 4);
				lambda = 1 / (cap->laser_plane[0] * u + cap->laser_plane[1] * v + cap->laser_plane[2] * w);
				campnt = (cv::Mat_<double>(1, 3) << lambda * u, lambda * v, lambda*w);//�J�������W�n�ʒu
				wldpnt = campnt * cap->Rots[0];//��]�s��������邱�Ƃ�World���W�n�ɒ����Ă���
				cap->Worlds.push_back(wldpnt.clone());
			}
		}

		cap->Worlds_Logs.push_back(cap->Worlds);
		//cap->Row_num_Logs.push_back(cap->row_num);

	}
	
	return 0;
}

//�P�����{�b�g�̃|�C���g�w��ړ�
void AxisToPoint(RS232c *axis, int *pointnum) {
	char command[256] = { '\0' };
	char buf[256];
	int rob_height;
	//axis->Read_CRLF(buf, 256);
	//printf(buf);//RUN
	sprintf(command, "@START%d.1\r\n", *pointnum);
	axis->Send(command);
	axis->Read_CRLF(buf, 256);
	//printf(buf);//RUN
	Sleep(5000);//���̎��Ԉȓ��ɓ����؂�
	axis->Read_CRLF(buf, 256);
	//printf(buf);//END
	//Sleep(1000);
	//axis.Send("@?D0.1\r\n");
	//axis.Read_CRLF(buf, 256);
	//printf(buf);//D0.1=~~
	//			/*axis.Send("@?D0.1\r\n");
	//			axis.Read_CRLF(buf, 256);
	//			printf(buf);*/
	//sscanf(buf, "D0.1=%d\r\n", &rob_height);
	//cap.height = cap.rob_ini_height + (float)rob_height * 0.01;
}
