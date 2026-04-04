/*****************************************************************************
*	Name: Socket.cpp
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Implementation of classes related to serial interface
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
#include "Serial.h"
#include <algorithm>
#include <string.h>

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
CSerialComm::CSerialComm()
{
    m_pCallbackConnect = NULL;
    m_pCallbackDisconnect = NULL;
    m_pCallbackSend = NULL;
    m_pCallbackRecv = NULL;

	m_bConnected = FALSE;
	
	/* default serial params */
    m_pSerialHandler = NULL;
    m_strSerialPort = "ttyACM0";
    m_eRs232flow = eRS232_FC_NONE;
    m_eRs232datasize = eRS232_BIT8;
    m_eRs232stopbit = eRS232_ONESTOPBIT;
    m_eRs232paritybit = eRS232_NOPARITY;
    m_eRs232baudrate = eRS232_CBR_9600;
    m_uRs232readtimeout = 1000;
    m_uRs232writetimeout = 1000;


	ZeroMemory(m_pSendBuffer);
	ZeroMemory(m_pRecvBuffer);
}

CSerialComm::~CSerialComm()
{
    
}
//////////////////////////////////////////////////////////////////////
BOOL 
CSerialComm::Init(TSTRING astrPort, INT anBaudRate)
{
    int nHandle = 0;
    struct termios stTermios;

    m_strSerialPort = astrPort;
    m_eRs232baudrate = eRS232_BAUDRATE(anBaudRate);

    TSTRING strPort = "/dev/" + m_strSerialPort;
    nHandle = open(strPort.c_str(), O_RDWR | O_NOCTTY);
    if (nHandle < 0)
    {
        DBG_LOG_ERROR("(%s) [%s] Open Failed with error code: %d - %s!", "CSerialComm", m_strSerialPort.c_str(), GetLastError(), strerror(GetLastError()));
        return FALSE;
    }

	tcgetattr(nHandle, &stTermios); // get current setting

	/* set input mode options */
	stTermios.c_iflag = IGNPAR; 	/* ignore parity errors */
	//stTermios.c_iflag = ICRNL; 	/* Map CR to NL */
	/* set output mode options */
	stTermios.c_oflag = 0;
	/* set local options, (no canonical, no echo, no isig, no echoe:echo erase character ) */
	stTermios.c_lflag = 0; // ICANON;
	//stTermios.c_lflag = ICANON; 	/* enable canonical input */
	//stTermios.c_lflag = ECHO; 	/* enable echoing of input */
	/* set control options */
	stTermios.c_cflag = CLOCAL | CREAD;
	switch (m_eRs232baudrate)
	{
	case eRS232_CBR_9600: 		stTermios.c_cflag |= B9600;		break;
	case eRS232_CBR_19200: 		stTermios.c_cflag |= B19200;	break;
	case eRS232_CBR_38400: 		stTermios.c_cflag |= B38400;	break;
	case eRS232_CBR_57600: 		stTermios.c_cflag |= B57600;	break;
	default:
	case eRS232_CBR_115200: 	stTermios.c_cflag |= B115200;	break;
	}

	stTermios.c_cflag &= ~CSIZE;
	switch (m_eRs232datasize)
	{
	case eRS232_BIT7: 			stTermios.c_cflag |= CS7;		break;
	default:
	case eRS232_BIT8: 			stTermios.c_cflag |= CS8;		break;
	}

	switch (m_eRs232paritybit)
	{
		/* no parity */
	case eRS232_NOPARITY: 		break;
		/* odd parity */
	case eRS232_ODDPARITY: 		stTermios.c_cflag |= PARODD;	break;
		/* even parity */
	default:
	case eRS232_EVENPARITY: 	stTermios.c_cflag |= PARENB;	break;
	}

	switch (m_eRs232stopbit)
	{
		/* 1 stop bit */
	default:
	case eRS232_ONESTOPBIT:		break;
		/* 2 stop bit */
	case eRS232_TWOSTOPBITS: 	stTermios.c_cflag |= CSTOPB;	break;
	}

	switch (m_eRs232flow)
	{
	default:
	case eRS232_FC_NONE: 		break;
	case eRS232_FC_DTRDSR: 		break;
	case eRS232_FC_XONXOFF: 	break;
	case eRS232_FC_RTSCTS: 		stTermios.c_iflag |= CRTSCTS;	break;
	}

	stTermios.c_cc[VTIME] = 0;
	stTermios.c_cc[VMIN] = 1; /* at least 1 char */

	tcflush(nHandle, TCIFLUSH);
	tcsetattr(nHandle, TCSANOW, &stTermios);

	m_pSerialHandler = new int();
	*(int*)m_pSerialHandler = nHandle;
	
	if (FALSE == StartThreads())
	{
		DBG_LOG_ERROR("(%s) [%s] Cannot start send and recv threads", "CSerialComm", m_strSerialPort.c_str());
		return FALSE;
	}
	m_bConnected = TRUE;

	if (m_pCallbackConnect)
	{
		m_pCallbackConnect((PVOID)this, NULL, NULL, NULL);
	}
    
	return TRUE;
}

void
CSerialComm::SetParams(LPTTYSTRUCT stParams)
{
	;
}

BOOL
CSerialComm::DeInit()
{
	int nHandle = 0;
	BOOL bRet = FALSE;

	if (!m_pSerialHandler)
		return bRet;

	nHandle = *(int*)m_pSerialHandler;
	if (nHandle > 0)
	{
		m_bConnected = FALSE;
		usleep(100); // for sync

		if (!close(nHandle))
		{
			bRet = TRUE;
		}
	}
	delete (int*)m_pSerialHandler;
	m_pSerialHandler = NULL;

	return bRet;
}

BOOL 
CSerialComm::StartThreads()
{
	if ((NULL != m_pRecvThread) || (NULL != m_pSendThread))
	{
		return FALSE;
	}
	m_pSendThread = new std::thread(&CSerialComm::proc_thread_send, this);
	m_pRecvThread = new std::thread(&CSerialComm::proc_thread_recv, this);
	return TRUE;
}

void
CSerialComm::TerminateRecvThread()
{
	m_bConnected = FALSE;
	if (m_pRecvThread)
	{
		if (m_pRecvThread->joinable())
			m_pRecvThread->join();

		delete m_pRecvThread;
		m_pRecvThread = nullptr;
	}
}

void
CSerialComm::TerminateSendThread()
{
	m_bConnected = FALSE;
	if (m_pSendThread)
	{
		if (m_pSendThread->joinable())
			m_pSendThread->join();

		delete m_pSendThread;
		m_pSendThread = nullptr;
	}
}

BOOL
CSerialComm::Send(const PVOID apBuffer, const UINT32 anLen)
{
	stSendData stData;

	stData.buffer = new char[anLen];
	memcpy(stData.buffer, apBuffer, anLen);
	stData.length = anLen;

	m_qSend.push(stData);
	return TRUE;
}

void
CSerialComm::proc_thread_recv()
{
	int nHandle = *(int*)m_pSerialHandler;
	int nErrorCode;

	size_t  lNumberOfBytesRecv = 0; 

	DBG_LOG_INFO("(%s) Receive Thread Initialized.", "CSerialComm", m_strSerialPort.c_str());
	while (IsConnected())
	{
		try
		{
			nErrorCode = 0;
			lNumberOfBytesRecv = read(nHandle, m_pRecvBuffer, MAX_RECV_BUFF_LEN);
			if (0 > lNumberOfBytesRecv)
			{
				nErrorCode = GetLastError();
				if (m_pCallbackRecv)
				{
					m_pCallbackRecv((PVOID)this, &nErrorCode, NULL, &lNumberOfBytesRecv);
				}

				DBG_LOG_ERROR("(%s) [%s] Read failed with error code: %d - %s!", "CSerialComm", m_strSerialPort.c_str(), GetLastError(), strerror(GetLastError()));
				
				m_bConnected = FALSE;
				return;
			}
			else
			{
				if (m_pCallbackRecv)
				{
					m_pCallbackRecv((PVOID)this, &nErrorCode, m_pRecvBuffer, &lNumberOfBytesRecv);
				}
			}
		}
		catch (...)
		{
			m_bConnected = FALSE;
			return;
		}
	}
}

void
CSerialComm::proc_thread_send()
{
	int nHandle = *(int*)m_pSerialHandler;
	size_t nSentBytes = 0;
	int nIndex, nErrorCode;
	UINT32 nBytesToSend;

	DBG_LOG_INFO("(%s) Send Thread Initialized.", "CSerialComm", m_strSerialPort.c_str());

	while (IsConnected())
	{
		if (!m_qSend.empty())
		{
			stSendData data = m_qSend.front();
			m_qSend.pop();

			nBytesToSend = data.length;
			nIndex = 0;

			while (nBytesToSend > 0)
			{
				nErrorCode = 0;
				nSentBytes = write(nHandle, &data.buffer[nIndex], nBytesToSend);
				if (nSentBytes < 0)
				{
					nErrorCode = errno;
					if (m_pCallbackSend)
					{
						m_pCallbackSend((PVOID)this, &nErrorCode, NULL, &nSentBytes);
					}
					
					DBG_LOG_ERROR("(%s) [%s] Read failed with error code: %d - %s!", "CSerialComm", m_strSerialPort.c_str(), GetLastError(), strerror(GetLastError()));
					
					break;
				}

				ZeroMemory(m_pSendBuffer);
				memcpy(m_pSendBuffer, &data.buffer[nIndex], nSentBytes);
				if (m_pCallbackSend)
				{
					m_pCallbackSend((PVOID)this, &nErrorCode, m_pSendBuffer, &nSentBytes);
				}
				nIndex += nBytesToSend;
				nBytesToSend -= nBytesToSend;
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
			}
			if (nSentBytes < 0)
			{
				break;
			}
			
		}
		else
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
	}

}