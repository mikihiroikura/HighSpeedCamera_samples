#pragma once
#ifndef _RS_232C_H_
#define _RS_232C_H_

#if _MSC_VER > 1000
#pragma once
#endif

#include <windows.h>

class RS232c {
private:
	HANDLE m_hComm;
	DWORD size;
	DCB	m_Dcb;
public:
	//各種パラメーターの設定後に接続します。 返り値　TRUE = 成功　FALSE = 失敗
	bool Connect(const char* PortNmae,  //ポート名を指定します　COM1 COM2など、初期値はCOM1
		int BaudRate,//ボーレートを指定します。初期値は 9600
		int ByteSize,//1バイトのビット数を指定します。初期値は 8
		int Parity,//パリティを指定します。パリティなし: NOPARITY
				   //偶数パリティ: EVENPARITY
				   //奇数パリティ: ODDPARITY
				   //初期値は、パリティなし: NOPARITY
		int StopBits,//ストップビット数を指定します。
					 //1ビット: ONESTOPBIT
					 //1.5ビット: ONE5STOPBITS
					 //2ビット: TWOSTOPBITS 
					 //初期値は　1ビット: ONESTOPBIT
		int RTS,		//RTSをON=RTS_CONTROL_ENABLE 初期値は無効です
		int DTR,		//DTRをON=DTR_CONTROL_ENABLE 初期値は無効です
		int ReadTimeOut,//受信時のタイムアウト　msで指定　初期値は　5000ms
		int WriteTimeOut//送信時のタイムアウト　msで指定　初期値は　20000ms
	);
	//文字列の受信 Read(char配列,読み込む文字数) 返り値　TRUE = 成功　FALSE = 失敗
	bool Read(char* Buff, int NumberOfCharactersToRead);
	//文字列の受信 Read_CRLF(char配列,バッファーサイズ) 返り値　TRUE = 成功　FALSE = 失敗
	//CRLFをキャッチしたときに受信完了します、読み込み文字列がバッファーサイズをあふれた場合は
	//その時点までの文字列しか受信できません
	bool Read_CRLF(char* Buff, int NumberOfCharactersToRead);
	//文字列を送信 Send(送信文字列) 返り値　TRUE = 成功　FALSE = 失敗
	bool Send(const char* word);
	//CHAR文字を送信 Send(送信文字列) 返り値　TRUE = 成功　FALSE = 失敗
	bool Send_CHAR(unsigned char wordchar);
	//接続の状態を取得します
	bool isLink();
	//デストラクタ　ポートを閉じます
	~RS232c();
};

bool RS232c::Connect(const char* PortNmae = "COM4",
	int BaudRate = 115200,
	int ByteSize = 8,
	int Parity = NOPARITY,
	int StopBits = ONESTOPBIT,
	int RTS = RTS_CONTROL_DISABLE,
	int DTR = DTR_CONTROL_DISABLE,
	int ReadTimeOut = 5000,
	int WriteTimeOut = 20000
)
{

	m_hComm = CreateFile(
		PortNmae,
		GENERIC_READ | GENERIC_WRITE,
		0,
		0,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL,
		0
	);

	if (m_hComm == INVALID_HANDLE_VALUE) return FALSE;

	//　イベントを使用しないようにセットする
	SetCommMask(m_hComm, 0);
	//　入出力バッファの許容量設定
	SetupComm(m_hComm, 2000L, 2000L);

	//　タイムアウト情報のセット
	COMMTIMEOUTS	m_CommTime;
	m_CommTime.ReadIntervalTimeout = 0xFFFFFFFF;
	m_CommTime.ReadTotalTimeoutMultiplier = 0;
	m_CommTime.ReadTotalTimeoutConstant = ReadTimeOut;
	m_CommTime.WriteTotalTimeoutMultiplier = 0;
	m_CommTime.WriteTotalTimeoutConstant = WriteTimeOut;

	//　タイムアウトの設定
	SetCommTimeouts(m_hComm, &m_CommTime);



	//　通信デバイス情報の取得
	GetCommState(m_hComm, &m_Dcb);

	//　通信デバイス情報の修正
	m_Dcb.DCBlength = sizeof(DCB);
	m_Dcb.fBinary = FALSE;
	m_Dcb.BaudRate = BaudRate;
	m_Dcb.ByteSize = ByteSize;
	m_Dcb.Parity = Parity;
	m_Dcb.StopBits = StopBits;
	m_Dcb.fRtsControl = RTS;
	m_Dcb.fDtrControl = DTR;
	m_Dcb.fDsrSensitivity = FALSE;
	m_Dcb.fAbortOnError = FALSE;
	m_Dcb.fNull = TRUE;
	m_Dcb.fParity = TRUE;
	m_Dcb.ErrorChar = 0x00;
	m_Dcb.fErrorChar = TRUE;


	SetCommState(m_hComm, &m_Dcb);
	if (!SetCommState(m_hComm, &m_Dcb)) return FALSE;

	return TRUE;
}

bool RS232c::Read(char* Buff, int NumberOfCharactersToRead) {
	for (int i = 0; i<NumberOfCharactersToRead; i++) {
		Buff[i] = '\0';
	}
	if (0 == ReadFile(m_hComm, Buff, NumberOfCharactersToRead, &size, NULL)) {
		return FALSE;
	}
	else {
		return TRUE;
	}
}

bool RS232c::Read_CRLF(char* Buff, int NumberOfCharactersToRead) {

	for (int i = 0; i<NumberOfCharactersToRead; i++) {
		Buff[i] = '\0';
	}
	char buf[10] = "";
	char beChar = 0x00;
	bool flag = TRUE;
	int c = 0;

	while ((!(buf[0] == 0x0A && beChar == 0x0D)) && c<NumberOfCharactersToRead && isLink()) {
		buf[0] = 0x00;

		if (0 == ReadFile(m_hComm, buf, 1, &size, NULL)) {
			flag = FALSE;
		}
		else {
			flag = TRUE;
		}
		if (buf[0] != 0x00 && flag) {
			if (c>1) beChar = Buff[c - 1];
			Buff[c++] = buf[0];
		}
		else {
			return FALSE;
		}
	}
	return TRUE;
}

bool RS232c::Send(const char* word) {
	if (0 == WriteFile(m_hComm, word, strlen(word), &size, NULL)) {
		return FALSE;
	}
	else {
		return TRUE;
	}
}

bool RS232c::Send_CHAR(unsigned char wordchar) {
	if (0 == WriteFile(m_hComm, &wordchar, 1, &size, NULL)) {
		return FALSE;
	}
	else {
		return TRUE;
	}
}

bool RS232c::isLink() {
	if (m_Dcb.fDtrControl == DTR_CONTROL_ENABLE) { //DTRが有効の場合のみ有効
		DWORD   dwSts;
		BOOL    fSuccess = GetCommModemStatus(m_hComm, &dwSts);    // 信号ステータスの取得
		if (!fSuccess) {
		}
		else {
			if (!(dwSts & MS_DSR_ON))	return FALSE;			//ＤＳＲ信号ＯＮ
																//if(dwSts & MS_CTS_ON) //CTSON
																//if(dwSts & MS_RING_ON) //RINGON
			if (!(dwSts & MS_RLSD_ON))  return FALSE;			//RLSDON
		}
	}
	return TRUE;
}


RS232c::~RS232c() {
	//ポートを閉じます
	CloseHandle(m_hComm);
}



#endif // _RS_232C_H_