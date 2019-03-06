//動体検出関数

#include "motion_detect.h"

//形状変化検出用パラメータ
vector<cv::Mat> Diffs;
cv::Mat oldimg, newimg;
cv::Mat diff;
cv::Mat	thrdiff;
int difframe = 50;//差分フレーム数
cv::Mat laserthr, laserthr2;//レーザー部分を抜くMask
cv::Rect roi(260, 170, 200, 200);
int meanval;//画像全体の平均輝度値
int detect_cnt_thr = 30;//形状変化検知のThreshold
int max_detection_num = 5;//この数値回数分連続で形状変化と検出したらOK
double ldiff;//レーザー距離差分
vector<double> ldiffs;
double ltarget;//検出後の形状変化中心部のPixel座標
double lthreshold = 50.0;//レーザー位置ずれの閾値
bool shapechange = 0;//形状変化検出フラグ(0：未検出，1：検出)
int mindiffno = 0;
int detect_cnt;//現在の検出処理カウンター番号
int detect_cnt_old;//一回前の検出処理カウンター番号




//連続difframe枚分のフレーム画像の差分画像から形状変化を検出
void ShapeChangeDetectionMultiFrame(Capture *cap, bool *flg) {
	while (*flg && !shapechange)
	{

		if (shapedetection_flg) {
			detect_cnt = cap->pic_cnt;
			if (detect_cnt > difframe) {//difframe分だけ処理カウンタが進んだ
				oldimg = cap->Pictures[detect_cnt - difframe].clone();
				newimg = cap->Pictures[detect_cnt - 1].clone();
			}
		}
		if (oldimg.data == NULL || newimg.data == NULL || detect_cnt <= detect_cnt_old) { continue; }
		detect_cnt_old = detect_cnt;
		//差分画像計算
		diff = abs(newimg - oldimg);

		//形状変化部分を差分画像から検出
		cv::threshold(oldimg, laserthr, 60, 255, cv::THRESH_BINARY);
		cv::threshold(newimg, laserthr2, 60, 255, cv::THRESH_BINARY);//レーザーが光っているところはThresholdかける
		cv::bitwise_or(laserthr, laserthr2, laserthr);
		cv::GaussianBlur(laserthr, laserthr, cv::Size(51, 17), 0);
		cv::threshold(laserthr(roi), laserthr, 0, 255, cv::THRESH_BINARY);
		cv::bitwise_not(laserthr, laserthr);
		meanval = (int)mean(diff)[0];//差分画像の平均画素値計算
		cv::bitwise_and(diff(roi), laserthr, thrmask);//レーザーが映ったところは消去
		cv::threshold(thrmask, thrmask, 15.0 + meanval, 255.0, cv::THRESH_BINARY);//15+画素値平均値の閾値超えを255に



																				  //形状変化検出画像の部分から0次モーメント(Pixel数)計算
		M = cv::moments(thrmask);
		if ((int)M.m00 / 255 > detect_cnt_thr && !(M.nu02>100 * M.nu20))//閾値以上の点数が形状変化点だったら
		{
			detection_num++;
			if (detection_num > max_detection_num) {
				//形状変化検出⇒レーザー移動
				//検出点群の重心のX方向@pix座標系計算
				//レーザー中心を形状変化中心に移動
				ltarget = M.m10 / M.m00 + roi.x;//x軸1次モーメント/0次モーメント=x軸重心
				{
					cv::AutoLock xlaserlock(mutex2);//Xlaser_logの排他制御
					float minldiff = 1000;
					for (int i = 0; i < Xlaser_log.size(); i++)
					{
						ldiff = abs(Xlaser_log[i] - ltarget);
						if (minldiff > ldiff) {
							minldiff = ldiff;
							mindiffno = i;
						}
					}
				}//アンロック
				unsigned char frameno = (unsigned char)mindiffno;
				mirror.Send("l");//フレーム番号送信前のフラグ
				mirror.Send_CHAR(frameno);//フレーム番号送信
				numchange_flg = true;//ログ出力のレーザー本数の変更フラグ
				shapechange = 1;//形状変化検出フラグ更新⇒終了
			}
		}
		else {
			detection_num = 0;
		}
		//ログ保存
		FrameNos.push_back(detect_cnt_old - 1);
		Detect_pixcnt_log.push_back((int)M.m00 / 255);
		Detect_num_log.push_back(detection_num);
	}
}