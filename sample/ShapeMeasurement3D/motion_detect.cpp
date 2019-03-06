//���̌��o�֐�

#include "motion_detect.h"

//�`��ω����o�p�p�����[�^
vector<cv::Mat> Diffs;
cv::Mat oldimg, newimg;
cv::Mat diff;
cv::Mat	thrdiff;
int difframe = 50;//�����t���[����
cv::Mat laserthr, laserthr2;//���[�U�[�����𔲂�Mask
cv::Rect roi(260, 170, 200, 200);
int meanval;//�摜�S�̂̕��ϋP�x�l
int detect_cnt_thr = 30;//�`��ω����m��Threshold
int max_detection_num = 5;//���̐��l�񐔕��A���Ō`��ω��ƌ��o������OK
double ldiff;//���[�U�[��������
vector<double> ldiffs;
double ltarget;//���o��̌`��ω����S����Pixel���W
double lthreshold = 50.0;//���[�U�[�ʒu�����臒l
bool shapechange = 0;//�`��ω����o�t���O(0�F�����o�C1�F���o)
int mindiffno = 0;
int detect_cnt;//���݂̌��o�����J�E���^�[�ԍ�
int detect_cnt_old;//���O�̌��o�����J�E���^�[�ԍ�




//�A��difframe�����̃t���[���摜�̍����摜����`��ω������o
void ShapeChangeDetectionMultiFrame(Capture *cap, bool *flg) {
	while (*flg && !shapechange)
	{

		if (shapedetection_flg) {
			detect_cnt = cap->pic_cnt;
			if (detect_cnt > difframe) {//difframe�����������J�E���^���i��
				oldimg = cap->Pictures[detect_cnt - difframe].clone();
				newimg = cap->Pictures[detect_cnt - 1].clone();
			}
		}
		if (oldimg.data == NULL || newimg.data == NULL || detect_cnt <= detect_cnt_old) { continue; }
		detect_cnt_old = detect_cnt;
		//�����摜�v�Z
		diff = abs(newimg - oldimg);

		//�`��ω������������摜���猟�o
		cv::threshold(oldimg, laserthr, 60, 255, cv::THRESH_BINARY);
		cv::threshold(newimg, laserthr2, 60, 255, cv::THRESH_BINARY);//���[�U�[�������Ă���Ƃ����Threshold������
		cv::bitwise_or(laserthr, laserthr2, laserthr);
		cv::GaussianBlur(laserthr, laserthr, cv::Size(51, 17), 0);
		cv::threshold(laserthr(roi), laserthr, 0, 255, cv::THRESH_BINARY);
		cv::bitwise_not(laserthr, laserthr);
		meanval = (int)mean(diff)[0];//�����摜�̕��ω�f�l�v�Z
		cv::bitwise_and(diff(roi), laserthr, thrmask);//���[�U�[���f�����Ƃ���͏���
		cv::threshold(thrmask, thrmask, 15.0 + meanval, 255.0, cv::THRESH_BINARY);//15+��f�l���ϒl��臒l������255��



																				  //�`��ω����o�摜�̕�������0�����[�����g(Pixel��)�v�Z
		M = cv::moments(thrmask);
		if ((int)M.m00 / 255 > detect_cnt_thr && !(M.nu02>100 * M.nu20))//臒l�ȏ�̓_�����`��ω��_��������
		{
			detection_num++;
			if (detection_num > max_detection_num) {
				//�`��ω����o�˃��[�U�[�ړ�
				//���o�_�Q�̏d�S��X����@pix���W�n�v�Z
				//���[�U�[���S���`��ω����S�Ɉړ�
				ltarget = M.m10 / M.m00 + roi.x;//x��1�����[�����g/0�����[�����g=x���d�S
				{
					cv::AutoLock xlaserlock(mutex2);//Xlaser_log�̔r������
					float minldiff = 1000;
					for (int i = 0; i < Xlaser_log.size(); i++)
					{
						ldiff = abs(Xlaser_log[i] - ltarget);
						if (minldiff > ldiff) {
							minldiff = ldiff;
							mindiffno = i;
						}
					}
				}//�A�����b�N
				unsigned char frameno = (unsigned char)mindiffno;
				mirror.Send("l");//�t���[���ԍ����M�O�̃t���O
				mirror.Send_CHAR(frameno);//�t���[���ԍ����M
				numchange_flg = true;//���O�o�͂̃��[�U�[�{���̕ύX�t���O
				shapechange = 1;//�`��ω����o�t���O�X�V�ˏI��
			}
		}
		else {
			detection_num = 0;
		}
		//���O�ۑ�
		FrameNos.push_back(detect_cnt_old - 1);
		Detect_pixcnt_log.push_back((int)M.m00 / 255);
		Detect_num_log.push_back(detection_num);
	}
}