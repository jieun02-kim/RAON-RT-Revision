/*****************************************************************************
*	Name: Serial.h
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Header for the classes related to serial interface
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
#ifndef _RBGM_SERIAL_COMM_
#define _RBGM_SERIAL_COMM_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h> 
#include <fcntl.h>
#include <functional>
#include <chrono>
#include <iostream>
#include <thread>
#include <queue>

#include <termios.h>
#include "Defines.h"

constexpr auto MAX_RECV_BUFF_LEN = 4095; // MOVE to Interface

enum eRS232_FLOW
{
	eRS232_FC_NONE = 0,
	eRS232_FC_DTRDSR = 1,
	eRS232_FC_RTSCTS = 2,
	eRS232_FC_XONXOFF = 4
};

enum eRS232_DATASIZE
{
	eRS232_BIT7 = 7,
	eRS232_BIT8 = 8
};

enum eRS232_BAUDRATE
{
	eRS232_CBR_1200 = 1200,
	eRS232_CBR_2400 = 2400,
	eRS232_CBR_4800 = 4800,
	eRS232_CBR_9600 = 9600,
	eRS232_CBR_14400 = 14400,
	eRS232_CBR_19200 = 19200,
	eRS232_CBR_38400 = 38400,
	eRS232_CBR_56000 = 56000,
	eRS232_CBR_57600 = 57600,
	eRS232_CBR_115200 = 115200
};

enum eRS232_STOPBIT
{
	eRS232_ONESTOPBIT = 0,
	eRS232_ONE5STOPBITS = 1,
	eRS232_TWOSTOPBITS = 2
};

enum eRS232_PARITY
{
	eRS232_NOPARITY = 0,
	eRS232_ODDPARITY = 1,
	eRS232_EVENPARITY = 2,
	eRS232_MARKPARITY = 3,
	eRS232_SPACEPARITY = 4
};

enum eRS232_ASCII_DEF
{
	eRS232_ASCII_BEL = 0x07,
	eRS232_ASCII_BS = 0x08,
	eRS232_ASCII_LF = 0x0A,
	eRS232_ASCII_CR = 0x0D,
	eRS232_ASCII_XON = 0x11,
	eRS232_ASCII_XOFF = 0x13,
	eRS232_ASCII_STX = 0x02,
	eRS232_ASCII_ETX = 0xFE
};

typedef struct _TTYSTRUCT
{
	TSTRING		strComPort;
	UINT8		btXonXoff;
	UINT8		btByteSize;
	UINT8		btFlowCtrl;
	UINT8		btParity;
	UINT8		btStopBits;
	UINT32		uBaudRate;
	UINT32		uReadTimeout;
	UINT32		uWriteTimeout;

}
__attribute__((packed)) TTYSTRUCT, *LPTTYSTRUCT;


#define SERIAL_PORT_MAX	15

struct stSendData
{
	char* buffer;
	UINT32	length;

	stSendData()
	{
		buffer = NULL;
		length = 0;
	}
};


class CSerialComm
{
public:
	CSerialComm();
	virtual ~CSerialComm();
	
	BOOL	Init						(TSTRING astrPort, INT anBaudRate = 9600);
	BOOL	DeInit						(	);
	BOOL	IsConnected					(	) { return m_bConnected; }
	BOOL	Send						(const PVOID apBuffer, const UINT32 anLen);

	void	SetParams					(LPTTYSTRUCT);
	void	RegisterCallbackConnect		(GLOBAL_CALLBACK_FN f) { m_pCallbackConnect = std::move(f); };
	void	RegisterCallbackDisconnect	(GLOBAL_CALLBACK_FN f) { m_pCallbackDisconnect = std::move(f); };
	void	RegisterCallbackSend		(GLOBAL_CALLBACK_FN f) { m_pCallbackSend = std::move(f); };
	void	RegisterCallbackRecv		(GLOBAL_CALLBACK_FN f) { m_pCallbackRecv = std::move(f); };

private:
	PVOID					m_pSerialHandler;
	TSTRING					m_strSerialPort;
	UINT8					m_eRs232flow;
	UINT8					m_eRs232datasize;
	UINT8					m_eRs232stopbit;
	UINT8					m_eRs232paritybit;
	UINT32					m_eRs232baudrate;
	UINT32					m_uRs232readtimeout;
	UINT32					m_uRs232writetimeout;

	BOOL					m_bConnected;

	BYTE					m_pSendBuffer[MAX_RECV_BUFF_LEN];
	PSTD_THREAD				m_pSendThread;
	BOOL					m_bIsSendThreadAlive = FALSE;

	std::queue<stSendData>	m_qSend;
	BYTE					m_pRecvBuffer[MAX_RECV_BUFF_LEN];
	PSTD_THREAD				m_pRecvThread;
	BOOL					m_bIsRecvThreadAlive = FALSE;

private:
	BOOL	StartThreads					(	);
	void	TerminateSendThread				(	);
	void	TerminateRecvThread				(	);
	void	proc_thread_recv				(	);
	void	proc_thread_send				(	);

private:
	GLOBAL_CALLBACK_FN m_pCallbackConnect;
	GLOBAL_CALLBACK_FN m_pCallbackDisconnect;
	GLOBAL_CALLBACK_FN m_pCallbackSend;
	GLOBAL_CALLBACK_FN m_pCallbackRecv;

};

#endif // _RBGM_SERIAL_COMM_