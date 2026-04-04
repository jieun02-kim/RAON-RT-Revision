/*****************************************************************************
*	Name: Socket.h
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Header for the classes related to socket interface
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
#ifndef _KIST_SOCKET_
#define _KIST_SOCKET_

#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <time.h> 
#include <fcntl.h>
#include <functional>
#include <chrono>
#include <iostream>

#include <thread>
#include <queue>

#include "Defines.h"


constexpr auto MAX_RECV_BUFF_LEN = 8192; // MOVE to Interface


#define SOCK_BUFF_LEN 4096

typedef struct sockaddr_in  SOCKADDR_IN;
typedef char* SOCK_IPADDR;


/* Socket Type */
typedef enum 
{
    eTCP	=	0,
    eUDP
} SOCKET_TYPE;

/* Socket Info */
typedef struct SOCKET_INFO
{
    SOCKET_TYPE     eSocketType;
    char            pcIpAddr[NI_MAXHOST];
    int             nPort;
    BOOL            bIsNonBlocking; 

    SOCKET_INFO()
    {
        eSocketType     = eTCP;
        nPort           = 7421;
        bIsNonBlocking  = FALSE; 
    }

}stSocketInfo;

typedef struct  SOCKCLIENT_INFO
{
    int                 nClientFd;
    SOCKADDR_IN         stClientAdd;
    socklen_t           nClientSockLen;
    BOOL                bIsInitialized;
    char                pcIpAddr[NI_MAXHOST];
    int                 nPort;
}stSocketClientInfo;


struct stSendData
{
	char*	buffer;
	UINT32	length;

	stSendData()
	{
		buffer = NULL;
		length = 0;
	}
};

class CSocket
{
   
public:
    CSocket();
    virtual ~CSocket();

private:
    BOOL                m_bIsSocketInit;

    GLOBAL_CALLBACK_FN  m_pCallbackOpen;
    GLOBAL_CALLBACK_FN  m_pCallbackAccept;
    GLOBAL_CALLBACK_FN  m_pCallbackReceive;
    GLOBAL_CALLBACK_FN  m_pCallbackSend;
    GLOBAL_CALLBACK_FN  m_pCallbackClose;


    // BYTE                m_pRecvBuffer[SOCK_BUFF_LEN];
    // BYTE                m_pSendBuffer[SOCK_BUFF_LEN];
    // STD_THREAD          m_pReceiveThread;
    // BOOL                m_bIsRecvThreadAlive  = FALSE;
    // void                proc_thread_recv( );

    // std::queue<stSendData>	m_queue_send;
    // STD_THREAD          m_pSendThread;
    // BOOL                m_bIsSendThreadAlive  = FALSE;
    // void                proc_thread_send( );

protected:
    int             m_nSocketFd;
    SOCKADDR_IN     m_stServerAddrIn;
    SOCKET_INFO     m_stSocketInfo;
    SOCKCLIENT_INFO m_stClientInfo;
    
public:
    int         GetFileDesc     (   ){return m_nSocketFd;};
    char*       GetIpAddr       (   ){return m_stSocketInfo.pcIpAddr;};
    int         GetPort         (   ){return m_stSocketInfo.nPort;};
    SOCKET_TYPE GetSocketType   (   ){return m_stSocketInfo.eSocketType;};
    BOOL        IsNonBlocking   (   ){return m_stSocketInfo.bIsNonBlocking;};
    BOOL        IsSocketInit    (   ){return m_bIsSocketInit;};
    BOOL        IsConnected     (   );

public:
    virtual BOOL                InitSocket      (SOCKET_TYPE);
    virtual BOOL                BindAddress     (SOCK_IPADDR, int);
    virtual BOOL                SetNonBlocking  (   );
    virtual BOOL                ListenToClients (int);
    virtual SOCKCLIENT_INFO     AcceptClient    (   );

public:
    // BOOL    OpenServer  (int, SOCK_IPADDR apcIpAddr = NULL, SOCKET_TYPE aeSocketType = eTCP, int nBackLog = 0, BOOL abIsNonBlocking = FALSE);
    // int     Send(const void*, const UINT32);
    // void    ShutDown    (   );
    void    Close       (   );

    void RegisterCallbackOpen(GLOBAL_CALLBACK_FN f){ m_pCallbackOpen = std::move(f); };
    void RegisterCallbackAccept(GLOBAL_CALLBACK_FN f){ m_pCallbackAccept = std::move(f); };
    void RegisterCallbackReceive(GLOBAL_CALLBACK_FN f){ m_pCallbackReceive = std::move(f); };
    void RegisterCallbackSend(GLOBAL_CALLBACK_FN f){ m_pCallbackSend = std::move(f); };
    void RegisterCallbackClose(GLOBAL_CALLBACK_FN f){ m_pCallbackClose = std::move(f); };
}; // CSocket


class CTcpClient
{

public:
    CTcpClient(SOCKCLIENT_INFO);
    virtual ~CTcpClient(  );

    void    StartListening (   );
    void    Close          (   );
    int     Send            (const void*, const UINT32);
    
    void SetConnected   (BOOL abIsConnected){m_bIsConnected =abIsConnected;};
    BOOL IsConnected    (   ){return m_bIsConnected;};

    TSTRING GetIPAddress    ();
    INT     GetPortNo       ();

    void RegisterCallbackConnect(GLOBAL_CALLBACK_FN f){ m_pCallbackConnect = std::move(f); };
    void RegisterCallbackReceive(GLOBAL_CALLBACK_FN f){ m_pCallbackReceive = std::move(f); };
    void RegisterCallbackSend(GLOBAL_CALLBACK_FN f){ m_pCallbackSend = std::move(f); };
    void RegisterCallbackClose(GLOBAL_CALLBACK_FN f){ m_pCallbackClose = std::move(f); };

private:
    BOOL                m_bIsConnected;
    SOCKCLIENT_INFO     m_stClientInfo;


    
    BYTE                m_pSendBuffer[SOCK_BUFF_LEN];
    PSTD_THREAD         m_pSendThread;
    BOOL                m_bIsSendThreadAlive  = FALSE;
    void                proc_thread_send( );
    void                TerminateRecvThread();
    std::queue<stSendData>	m_queue_send;    

    BYTE                m_pRecvBuffer[SOCK_BUFF_LEN];
    PSTD_THREAD         m_pRecvThread;
    BOOL                m_bIsRecvThreadAlive  = FALSE;
    void                proc_thread_recv( );
    void                TerminateSendThread();

    GLOBAL_CALLBACK_FN  m_pCallbackConnect;
    GLOBAL_CALLBACK_FN  m_pCallbackReceive;
    GLOBAL_CALLBACK_FN  m_pCallbackSend;
    GLOBAL_CALLBACK_FN  m_pCallbackClose;
};

class CTcpServer
{
public:
    CTcpServer();
    virtual ~CTcpServer();


    BOOL    Start           (int anPort, int anBackLog = 1, BOOL abIsNonBlocking = FALSE);
    BOOL    Stop            (   );
    BOOL    IsConnected     (   );
    int     SendToClients   (const void*, const UINT32);

    void RegisterCallbackStart(GLOBAL_CALLBACK_FN f){ m_pCallbackStart = std::move(f); };
    void RegisterCallbackAccept(GLOBAL_CALLBACK_FN f){ m_pCallbackAccept = std::move(f); };
    void RegisterCallbackReceive(GLOBAL_CALLBACK_FN f){ m_pCallbackReceive = std::move(f); };
    void RegisterCallbackSend(GLOBAL_CALLBACK_FN f){ m_pCallbackSend = std::move(f); };
    void RegisterCallbackStop(GLOBAL_CALLBACK_FN f){ m_pCallbackStop = std::move(f); };
    void RegisterCallbackClientDisconnect(GLOBAL_CALLBACK_FN f){ m_pCallbackClientDisconnect = std::move(f); };


private:
    CSocket           *m_pSocket;
    
    PSTD_THREAD         m_pAcceptThread;
    BOOL                m_bIsAcceptThreadAlive;
    void                proc_thread_accept( );
    void                OnFaultAcceptThread();
    void                TerminateAcceptThread();

    PSTD_THREAD         m_pDeadClientRemoverThread;
    BOOL                m_bIsDeadClientRemoverThreadAlive;
    void                proc_thread_dead_client_remover();
    void                TerminateDeadClientRemoverThread();

protected:
    virtual void OnClientConnect(void *, void *, void *,void *);
    virtual void OnClientReceive(void *, void *, void *,void *);
    virtual void OnClientSend(void *, void *, void *,void *);
    virtual void OnClientClose(void *, void *, void *,void *);

private:
    std::vector<CTcpClient*> m_vTcpClient;
    GLOBAL_CALLBACK_FN  m_pCallbackStart;
    GLOBAL_CALLBACK_FN  m_pCallbackAccept;
    GLOBAL_CALLBACK_FN  m_pCallbackReceive;
    GLOBAL_CALLBACK_FN  m_pCallbackSend;
    GLOBAL_CALLBACK_FN  m_pCallbackStop;
    GLOBAL_CALLBACK_FN  m_pCallbackClientDisconnect;

};









#endif // _KIST_SOCKET_