
#include <opencv2/opencv.hpp>
#include <thread>
#include <vector>
#include <fstream>

#ifdef _DEBUG
#define CAMERA_EXT "d.lib"
#else
#define CAMERA_EXT ".lib"
#endif

#define STR_EXP(__A) #__A
#define STR(__A) STR_EXP(__A)
#define CV_VER STR(CV_VERSION_MAJOR) STR(CV_VERSION_MINOR) STR(CV_VERSION_REVISION)
#pragma comment(lib, "opencv_world" CV_VER CAMERA_EXT)

#define MAX_

#include <HSC/baslerClass.hpp>
#pragma comment(lib, "BaslerLib" CAMERA_EXT)

using namespace std;

struct Capture
{
	bool flag;
	basler cam;
	cv::Mat in_img;
	cv::Mat out_img;
	float rob_ini_height = 0.3f;
	float height_interval = 0.01f;
	float height = rob_ini_height;
	vector<double> CoGs;
};

//�v���g�^�C�v�錾
void capthread(Capture *cap);
void ChangeRobHeight(Capture *cap);
void HeightCalibration(Capture *cap, ofstream &fout);

int main() {

	int width = 640;
	int height = 480;
	float  fps = 750.0f;
	float gain = 1.0f;

	ofstream fout("light_section_result.csv");

	Capture cap;
	cap.in_img = (cv::Mat(height, width, CV_8UC1, cv::Scalar::all(255)));
	cap.flag = true;
	

	cap.cam.connect(0);
	cap.cam.setParam(paramTypeCamera::paramInt::WIDTH, width);
	cap.cam.setParam(paramTypeCamera::paramInt::HEIGHT, height);
	cap.CoGs.resize(height, 0);
	cap.cam.setParam(paramTypeBasler::FastMode::SensorReadoutModeFast);
	cap.cam.setParam(paramTypeBasler::Param::ExposureTime, 1280.0f);
	cap.cam.setParam(paramTypeCamera::paramFloat::FPS, fps);
	cap.cam.setParam(paramTypeCamera::paramFloat::GAIN, gain);
	cap.cam.setParam(paramTypeBasler::AcquisitionMode::EnableAcquisitionFrameRate);//�t���[�����[�g���Œ肵�ăp�����[�^���߂�
	cap.cam.setParam(paramTypeBasler::GrabStrategy::OneByOne);//�B���摜���̂ĂȂ��Œx��Ă�1��������
	/*cap.cam.setParam(paramTypeBasler::CaptureType::BayerBGGrab);
	cap.cam.setParam(paramTypeBasler::CaptureType::ColorBGRGrab);*/
	cap.cam.setParam(paramTypeBasler::CaptureType::MonocroGrab);
	cap.cam.parameter_all_print();

	cap.cam.start();
	
	//�����ɒP�����{�b�g�̍��x��ύX����Thread�����//
	/*thread thr2(ChangeRobHeight,&cap);
	thread thr(capthread,&cap);*/
	

	while (1) {
		//HeightCalibration(&cap, fout);
		cap.cam.captureFrame(cap.in_img.data);
		cv::imshow("img", cap.in_img);
		int key = cv::waitKey(1);
		if (key == 'q')break;
	}
	cap.flag = false;
	/*if (thr.joinable())thr.join();
	if (thr2.joinable())thr2.join();*/

	cap.cam.stop();
	cap.cam.disconnect();
}

void capthread(Capture *cap) {
	while (cap->flag) {
		/*double threshold = 128.0;
		double max_val = 255.0;*/
		cap->cam.captureFrame(cap->in_img.data);
		//���ؒf�@�摜����
		cv::threshold(cap->in_img, cap->out_img, 128.0, 255.0, cv::THRESH_BINARY);
		cv::bitwise_and(cap->in_img, cap->out_img, cap->out_img);
		vector<double> cogs;
		double moment,mass,cog;
		for (size_t i = 0; i < cap->out_img.rows; i++){
			moment = 0.0;
			mass = 0.0;
			for (size_t j = 0; j < cap->out_img.cols; j++){
				mass += cap->out_img.data[i*cap->out_img.cols + j];
				moment += j*cap->out_img.data[i*cap->out_img.cols + j];
			}
			cog = moment / mass;
			cogs.push_back(cog);
		}
		memcpy(&cap->CoGs, &cogs, sizeof(cap->CoGs));
		cogs.clear();
	}
}

void ChangeRobHeight(Capture *cap) {
	int max_rob_height = 700;//mm
	for (int i = 0; i < max_rob_height; i++){
		cap->height = cap->rob_ini_height + (double)i * 0.001;
	}
}

//�܂������ɃG���[����
void HeightCalibration(Capture *cap,ofstream &fout) {
	//�J��������t���[���擾
	cap->cam.captureFrame(cap->in_img.data);
	//���ؒf�@�ɂ��s���Ƃ̋P�x�d�S�̌v�Z
	cv::threshold(cap->in_img, cap->out_img, 128.0, 255.0, cv::THRESH_BINARY);
	cv::bitwise_and(cap->in_img, cap->out_img, cap->out_img);
	vector<double> cogs;
	double moment, mass, cog;
	for (size_t i = 0; i < cap->out_img.rows; i++) {
		moment = 0.0;
		mass = 0.0;
		for (size_t j = 0; j < cap->out_img.cols; j++) {
			mass += cap->out_img.data[i*cap->out_img.cols + j];
			moment += j * cap->out_img.data[i*cap->out_img.cols + j];
		}
		cog = moment / mass;
		cogs.push_back(cog);
	}
	memcpy(&cap->CoGs, &cogs, sizeof(cap->CoGs));//Capture�\���̂�CoGs�ɕۑ�
	cogs.clear();
	//�o�̓t�@�C���Ɍ��ʂ�ۑ�����
	fout << cap->height << "," <<endl;
	for (size_t i = 0; i < cap->CoGs.size(); i++){
		fout << cap->CoGs[i] << ",";
	}
	fout << endl;
	//�P�����{�b�g�̍����ύX
	cap->height += cap->height_interval;
	//�����ɒP�����{�b�g�𐧌䂷��֐�������//
}