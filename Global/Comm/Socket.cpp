/*****************************************************************************
*	Name: Socket.cpp
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Implementation of classes related to socket interface
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
#include <Socket.h>
#include <algorithm>

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
CSocket::CSocket()
{
    m_pCallbackAccept = NULL;
    m_pCallbackOpen = NULL;
    m_pCallbackReceive = NULL;
    m_pCallbackSend = NULL;

    m_bIsSocketInit = FALSE;
}

CSocket::~CSocket()
{
    
}
//////////////////////////////////////////////////////////////////////
BOOL
CSocket::InitSocket  (SOCKET_TYPE aeSocketType)
{
    char szMsg[__MAX_MESSAGE__] = "";
    BOOL bRet = TRUE;

    int nType = SOCK_DGRAM; // udp
    if ( eTCP == aeSocketType )
        nType = SOCK_STREAM;
    
    /* create socket file descriptor */
    if ( 0 > (m_nSocketFd = socket(AF_INET, nType, 0)) )
    {
        bRet = FALSE; // no effect
        sprintf(szMsg, "Init Socket, errno: [%d] %s", errno, strerror(errno));    
        throw std::runtime_error(szMsg);
    }

    m_stSocketInfo.eSocketType = aeSocketType;
    m_bIsSocketInit = bRet;

    /* set socket for reuse (otherwise might have to wait 4 minutes every time socket is closed) */
    const int option = 1;
    setsockopt(m_nSocketFd, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option));

    return bRet;
}

BOOL
CSocket::BindAddress  (SOCK_IPADDR apcIpAddr, int anPort)
{
    char szMsg[__MAX_MESSAGE__] = "";
    BOOL bRet = TRUE;

    if ( FALSE == m_bIsSocketInit )
    {
        sprintf(szMsg, "Bind before init");    
        throw std::runtime_error(szMsg);
    }
    
    /* configure socket (ip address and port) */
    ZeroMemoryE(&m_stServerAddrIn, sizeof(m_stServerAddrIn));

    m_stServerAddrIn.sin_family = AF_INET;
    m_stServerAddrIn.sin_addr.s_addr  = ( NULL == apcIpAddr ) ? htonl(INADDR_ANY) : inet_addr(apcIpAddr); // use 0.0.0.0 if NULL
    m_stServerAddrIn.sin_port = htons((UINT16)(anPort));

    /* bind socket */
    if ( 0 > ( bind(m_nSocketFd, (const struct sockaddr*)&m_stServerAddrIn, sizeof(m_stServerAddrIn))))
    {
        bRet = FALSE;
        sprintf(szMsg, "Bind Address, errno: [%d] %s", errno, strerror(errno));    
        throw std::runtime_error(szMsg);
    }
    
    char pcHost[NI_MAXHOST];
    char pcService[NI_MAXSERV];
    
    if ( 0 == getnameinfo((sockaddr*)&m_stServerAddrIn, sizeof(m_stServerAddrIn), pcHost, NI_MAXHOST, pcService, NI_MAXSERV, 0) )
        DBG_LOG_INFO("[CSocket] Initialized IPADDR: %s, PORT: %s", pcHost, pcService);
    else
    {
        inet_ntop(AF_INET, &m_stServerAddrIn.sin_addr, pcHost, NI_MAXHOST);
        DBG_LOG_INFO("[CSocket] Initialized IPADDR: %s, PORT: %u", pcHost, ntohs(m_stServerAddrIn.sin_port));
    }

    strcpy(m_stSocketInfo.pcIpAddr, pcHost);
    m_stSocketInfo.nPort = anPort;
    return bRet;
}

BOOL
CSocket::SetNonBlocking  (   )
{
    char szMsg[__MAX_MESSAGE__] = "";
    BOOL bRet = TRUE;

    if ( FALSE == m_bIsSocketInit )
    {
        sprintf(szMsg, "Set Non Blocking before Init");    
        throw std::runtime_error(szMsg);
    }
    
    if ( 0 > ( fcntl(m_nSocketFd, F_SETFL, O_NONBLOCK) ))
    {
        bRet = FALSE;
        sprintf(szMsg, "Set Non Blocking, errno: [%d] %s", errno, strerror(errno));    
        throw std::runtime_error(szMsg);
    }
    
    m_stSocketInfo.bIsNonBlocking = bRet;
    return bRet;
}

BOOL
CSocket::ListenToClients(int anBackLog)
{
    char szMsg[__MAX_MESSAGE__] = "";
    BOOL bRet = TRUE;
    
    if ( FALSE == m_bIsSocketInit && eTCP != m_stSocketInfo.eSocketType )
    {
        sprintf(szMsg, "Listen before Init, or not TCP");    
        throw std::runtime_error(szMsg);
    }

    if ( 0 > ( listen(m_nSocketFd, anBackLog)))
    {
        bRet = FALSE;
        sprintf(szMsg, "Listen client, errno: [%d] %s", errno, strerror(errno));    
        throw std::runtime_error(szMsg);
    }
    return bRet;
}
/*
* Accept and handle new client socket. To handle multiple clients, user must
* call this function in a loop to enable the acceptance of more than one.
* If timeout argument equal 0, this function is executed in blocking mode.
* If timeout argument is > 0 then this function is executed in non-blocking
* mode (async) and will quit after timeout seconds if no client tried to connect.
* Return accepted client IP, or throw error if failed
*/
SOCKCLIENT_INFO
CSocket::AcceptClient()
{
    BOOL bRet = TRUE;
    char szMsg[__MAX_MESSAGE__] = "";
    SOCKCLIENT_INFO stsockclient;

    if ( FALSE == m_bIsSocketInit && eTCP != m_stSocketInfo.eSocketType )
    {
        bRet = FALSE;
        sprintf(szMsg, "Accept before Init, or not TCP");    
        throw std::runtime_error(szMsg);
    }

    int         nClientFd;
    SOCKADDR_IN stClientAddr;
    socklen_t   nClientSockLen = sizeof(stClientAddr);
    if ( 0 > (nClientFd = accept(m_nSocketFd, (sockaddr*)&stClientAddr, &nClientSockLen)))
    {
        bRet = FALSE;
        sprintf(szMsg, "Accept client, errno: [%d] %s", errno, strerror(errno));    
        throw std::runtime_error(szMsg);
    }

    /* Get Client Information */
    unsigned short portNumber;
    char pcHost[NI_MAXHOST];
    /*char pcService[NI_MAXSERV];
    if ( 0 == getnameinfo((sockaddr*)&stClientAddr, sizeof(stClientAddr), pcHost, NI_MAXHOST, pcService, NI_MAXSERV, 0) )
    {
        DBG_LOG_INFO("[CSocket] Initialized IPADDR: %s, PORT: %s", pcHost, pcService);
        portNumber = (unsigned short)(atoi(pcService));
    }*/
    inet_ntop(AF_INET, &stClientAddr.sin_addr, pcHost, NI_MAXHOST);
    portNumber = ntohs(stClientAddr.sin_port);

    stsockclient.nClientFd = nClientFd;
    stsockclient.nClientSockLen = nClientSockLen;
    stsockclient.stClientAdd = stClientAddr;
    strcpy(stsockclient.pcIpAddr, pcHost);
    stsockclient.nPort = portNumber;
    stsockclient.bIsInitialized = bRet;

    return stsockclient;
}

void
CSocket::Close (  )
{
    close(m_nSocketFd);
    
    if (m_pCallbackClose)
            m_pCallbackClose(NULL, NULL, NULL, NULL);
}

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
CTcpClient::CTcpClient(SOCKCLIENT_INFO aClientInfo)
{
    m_stClientInfo = aClientInfo;
    SetConnected(FALSE);

    ZeroMemory(m_pSendBuffer);
    ZeroMemory(m_pRecvBuffer);
}

CTcpClient::~CTcpClient()
{
    
}
//////////////////////////////////////////////////////////////////////
void
CTcpClient::StartListening(   )
{
    SetConnected(TRUE);
    m_pRecvThread = new std::thread(&CTcpClient::proc_thread_recv, this);
    m_pSendThread = new std::thread(&CTcpClient::proc_thread_send, this);

    if (m_pCallbackConnect)
        m_pCallbackConnect((PVOID) this, NULL, NULL, NULL);
}

int
CTcpClient::Send(const void* apBuffer, const UINT32 anLength)
{
	stSendData data;
	data.buffer = new char[anLength];
	memcpy(data.buffer, apBuffer, anLength);
	data.length = anLength;

	m_queue_send.push(data);
	return 0;
}

TSTRING
CTcpClient::GetIPAddress()
{
    return TSTRING(m_stClientInfo.pcIpAddr);
}

int
CTcpClient::GetPortNo()
{
    return m_stClientInfo.nPort;
}

void
CTcpClient::proc_thread_send()
{
	size_t nSentBytes = 0;
    int nIndex, nErrorCode;
	UINT32 nBytesToSend;

	while (IsConnected())
	{
		if (!m_queue_send.empty())
		{
			stSendData data = m_queue_send.front();
			m_queue_send.pop();

			nBytesToSend = data.length;
			nIndex = 0;

			while (nBytesToSend > 0)
			{
				nErrorCode = 0;
				nSentBytes = send(m_stClientInfo.nClientFd, &data.buffer[nIndex], nBytesToSend, 0);
				if (nSentBytes < 0)
				{
                    
					nErrorCode = errno;
                    if (m_pCallbackSend)
                    {
                        m_pCallbackSend((PVOID)this, &nErrorCode, NULL, &nSentBytes);
                        break;
                    }
				}

				ZeroMemory(m_pSendBuffer);
				memcpy(m_pSendBuffer, &data.buffer[nIndex], nSentBytes);
				if (m_pCallbackSend)
				{
					m_pCallbackSend((PVOID)this, &nErrorCode, m_pSendBuffer, &nSentBytes);
				}
				nIndex += nBytesToSend;
				nBytesToSend -= nBytesToSend;
			}

			if (nSentBytes < 0)
				break;
		}
		else
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
	}
}

void 
CTcpClient::TerminateSendThread(  )
{
    SetConnected(FALSE);
    if (m_pSendThread)
    {
        if (m_pSendThread->joinable())
            m_pSendThread->join();

        delete m_pSendThread;
        m_pSendThread = nullptr;
    }
}

void
CTcpClient::proc_thread_recv()
{
    int nRecvFlag = 0;
    int nErrorCode = 0;
    size_t  lNumberOfBytesRecv = 0; /* size_t = uint64_t */

    while(IsConnected())
    {
        try
        {
            lNumberOfBytesRecv = recv(m_stClientInfo.nClientFd, m_pRecvBuffer, SOCK_BUFF_LEN, nRecvFlag);
            if (1 > lNumberOfBytesRecv)
            {
                nErrorCode = errno; // get last error code
                if (m_pCallbackReceive)
                {
                    m_pCallbackReceive((PVOID)this, &nErrorCode, NULL, &lNumberOfBytesRecv);
                }
                SetConnected(FALSE);
                return;
            }
            else
            {
                if (m_pCallbackReceive)
                {
                    m_pCallbackReceive((PVOID)this, &nErrorCode, m_pRecvBuffer, &lNumberOfBytesRecv);
                }
            }
        }
        catch (...)
        {
            SetConnected(FALSE);
            return;
        }
    }
}

void 
CTcpClient::TerminateRecvThread(  )
{
    SetConnected(FALSE);
    if (m_pRecvThread)
    {
        if (m_pRecvThread->joinable())
            m_pRecvThread->join();

        delete m_pRecvThread;
        m_pRecvThread = nullptr;
    }
}

void
CTcpClient::Close(    )
{
    TerminateRecvThread(    );
    TerminateSendThread(    );
    
    if (m_pCallbackClose)
        m_pCallbackClose((PVOID)this, NULL, NULL, NULL);

    close(m_stClientInfo.nClientFd);

}


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
CTcpServer::CTcpServer()
{
    m_pCallbackStart = NULL;
    m_pCallbackAccept = NULL;
    m_pCallbackReceive = NULL;
    m_pCallbackSend = NULL;
    m_pCallbackStop = NULL;


    m_bIsAcceptThreadAlive = TRUE;
    m_bIsDeadClientRemoverThreadAlive = TRUE;

    m_vTcpClient.reserve(10);
    m_pSocket = new CSocket( );
}

CTcpServer::~CTcpServer()
{
    
}
//////////////////////////////////////////////////////////////////////
BOOL
CTcpServer::Start (int anPort, int anBackLog /*=1*/, BOOL abIsNonBlocking /*=FALSE*/)
{
    try
    {
        m_pSocket->InitSocket(eTCP);
        m_pSocket->BindAddress(NULL, anPort);
        m_pSocket->ListenToClients(anBackLog);
        if ( TRUE == abIsNonBlocking ) 
            m_pSocket->SetNonBlocking(  );
    }
    catch(const std::runtime_error& e)
    {
        Stop();
        DBG_LOG_ERROR("[CTcpServer] %s", e.what());
        return FALSE;
    }
    catch(...)
    {
        Stop();
        DBG_LOG_ERROR("[CTcpServer] UnknownError");
        return FALSE;
    }

    /* Start Accept Thread */
    m_pAcceptThread = new std::thread(&CTcpServer::proc_thread_accept, this);

    /* Start Dead Client Remover Thread */ 
    m_pDeadClientRemoverThread = new std::thread(&CTcpServer::proc_thread_dead_client_remover, this);

    return TRUE;
}

BOOL
CTcpServer::IsConnected(  )
{
    if (!m_vTcpClient.empty())
        return TRUE;

    return FALSE;
}


int
CTcpServer::SendToClients (const void* apBuffer, const UINT32 anLength)
{
    for (CTcpClient *client : m_vTcpClient)
    {
        client->Send(apBuffer, anLength);
    }
    return 0;
}

void
CTcpServer::OnClientConnect (void *apClient, void *apnError, void *apBuffer,void *apnLength)
{
    if (m_pCallbackAccept)
        m_pCallbackAccept(apClient, apnError, apBuffer, apnLength);
}

void
CTcpServer::OnClientReceive(void *apClient, void *apnError, void *apBuffer,void *apnLength)
{
    if (m_pCallbackReceive)
        m_pCallbackReceive(apClient, apnError, apBuffer, apnLength);
}

void
CTcpServer::OnClientSend(void *apClient, void *apnError, void *apBuffer,void *apnLength)
{
    if (m_pCallbackSend)
        m_pCallbackSend(apClient, apnError, apBuffer, apnLength);
}

void
CTcpServer::OnClientClose(void *apClient, void *apnError, void *apBuffer,void *apnLength)
{
    if (m_pCallbackClientDisconnect)
        m_pCallbackClientDisconnect(NULL, NULL, NULL, NULL);
}

void 
CTcpServer::proc_thread_accept(   )
{
    while( TRUE == m_bIsAcceptThreadAlive )
    {
        try
        {
            if (m_pSocket)
            {
                SOCKCLIENT_INFO newClientInfo = m_pSocket->AcceptClient(    ); /* blocking */
                auto newClient = new CTcpClient(newClientInfo);
                m_vTcpClient.push_back(newClient);

                newClient->RegisterCallbackConnect(std::bind(&CTcpServer::OnClientConnect, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
                newClient->RegisterCallbackSend(std::bind(&CTcpServer::OnClientSend, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
                newClient->RegisterCallbackReceive(std::bind(&CTcpServer::OnClientReceive, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
                newClient->RegisterCallbackClose(std::bind(&CTcpServer::OnClientClose, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
                
                newClient->StartListening();
                
            }
            else
            {
                OnFaultAcceptThread( );
            }
        }
        catch(const std::runtime_error& e)
        {
            //DBG_LOG_ERROR("[CTcpServer] %s", e.what());
            std::this_thread::sleep_for(std::chrono::milliseconds(1)); // not really needed because accept is blocking
            //OnFaultAcceptThread(  );
        }
    }
}

void 
CTcpServer::OnFaultAcceptThread( )
{
    TerminateAcceptThread();
}

void
CTcpServer::TerminateAcceptThread(    )
{
    if(m_pAcceptThread)
    {
        m_bIsAcceptThreadAlive = FALSE;
        if (m_pAcceptThread->joinable())
            m_pAcceptThread->join();

        delete m_pAcceptThread;
        m_pAcceptThread = nullptr;
    }
}

void
CTcpServer::proc_thread_dead_client_remover( )
{
    std::vector<CTcpClient*>::const_iterator clientToRemove;
    while (TRUE == m_bIsDeadClientRemoverThreadAlive)
    {
        do 
        {
            clientToRemove = std::find_if(m_vTcpClient.begin(), m_vTcpClient.end(),[](CTcpClient *client) { return !client->IsConnected(); });
            if (clientToRemove != m_vTcpClient.end()) 
            {
                (*clientToRemove)->Close();
                delete *clientToRemove;
                m_vTcpClient.erase(clientToRemove);
            }
        } while (clientToRemove != m_vTcpClient.end());
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void
CTcpServer::TerminateDeadClientRemoverThread(     )
{
    if (m_pDeadClientRemoverThread)
    {
        m_bIsDeadClientRemoverThreadAlive = FALSE;
        if (m_pDeadClientRemoverThread->joinable())
            m_pDeadClientRemoverThread->join();

        delete m_pDeadClientRemoverThread;
        m_pDeadClientRemoverThread = nullptr;
    }
}


BOOL
CTcpServer::Stop( )
{
    
    TerminateDeadClientRemoverThread( );
    TerminateAcceptThread(  );

    
    for (CTcpClient * client : m_vTcpClient)
    {
        try
        {
            client->Close();
        }
        catch(const std::runtime_error& e)
        {
            DBG_LOG_ERROR("[CTcpServer] Close Error. %s", e.what());
        }
    }
    m_vTcpClient.clear( );

    if (m_pSocket)
        m_pSocket->Close(   );
    

    return TRUE;
}


