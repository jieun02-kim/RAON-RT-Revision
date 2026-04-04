/*****************************************************************************
*	Name: EcatInterface.cpp
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Implementation of the CEcatInterface class
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
#include "EcatInterface.h"


static const BYTEARRAY pbHead = { 0x02, 0x5B };
static const BYTEARRAY pbTail = { 0x3E, 0x5D };

/////////////////////////////////////////////////////////////////////
BYTEARRAY
ConvertIntToBytearray(int anValue)
{
	BYTEARRAY arr{};
	arr.push_back((uint8_t)((anValue >> 24) & 0xFF));
	arr.push_back((uint8_t)((anValue >> 16) & 0xFF));
	arr.push_back((uint8_t)((anValue >> 8) & 0xFF));
	arr.push_back((uint8_t)(anValue & 0xFF));

	return arr;
}

BYTEARRAY
ConvertUShortToByteArray(unsigned short anValue)
{
	BYTEARRAY arr{};
	arr.push_back((uint8_t)((anValue >> 8) & 0xFF));
	arr.push_back((uint8_t)(anValue & 0xFF));

	return arr;
}
/////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
CEcatInterface::CEcatInterface()
{
    m_pSocket   = NULL;
    m_pcEcatMaster = NULL;

    m_nSpinId = 0;
    m_crc16 = CRC16_ARC();
	m_pRecvBuffer.clear();   
}

CEcatInterface::~CEcatInterface()
{
    
}
//////////////////////////////////////////////////////////////////////

BOOL
CEcatInterface::Init(CEcatMasterBase* apcEcatMaster, int anPort /*=(int)DEFAULT_PORT*/)
{
    BOOL bRet = TRUE;

    DeInit();

    m_pcEcatMaster = apcEcatMaster;
    if (!m_pcEcatMaster)
    {
        DBG_LOG_ERROR("Create EtherCAT Master first!");
        bRet = FALSE;
    }
    
    m_pSocket = new CTcpServer();
    
    if ( m_pSocket )
    {
        m_pSocket->RegisterCallbackStart(std::bind(&CEcatInterface::OnSocketOpen, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
		m_pSocket->RegisterCallbackReceive(std::bind(&CEcatInterface::OnSocketReceive, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
		m_pSocket->RegisterCallbackSend(std::bind(&CEcatInterface::OnSocketSend, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
		m_pSocket->RegisterCallbackStop(std::bind(&CEcatInterface::OnSocketClose, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
		m_pSocket->RegisterCallbackAccept(std::bind(&CEcatInterface::OnSocketAccept, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
		m_pSocket->RegisterCallbackClientDisconnect(std::bind(&CEcatInterface::OnSocketClientDisconnect, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
    }

    if (FALSE == m_pSocket->Start(anPort))
    {
        DeInit();
        bRet = FALSE;
    }

    return bRet;
}

BOOL
CEcatInterface::DeInit(   )
{
    BOOL bRet = TRUE;
    if ( m_pSocket )
    {
        m_pSocket->Stop();
        delete m_pSocket;
        m_pSocket = NULL;
    }

    return bRet;
}

BOOL
CEcatInterface::IsConnected(  )
{
    if ( m_pSocket )
        return m_pSocket->IsConnected();

    return FALSE;
}

void
CEcatInterface::OnSocketReceive(PVOID apSocket, PVOID apnError, PVOID apBuffer, PVOID apnLength)
{
    int nErrorCode = *(int *) apnError;
    int nLength = *(int *) apnLength;
    
    if (nErrorCode == 0)
    {
        if ( m_pRecvBuffer.size() >= MAX_RECV_BUFF_LEN )
            m_pRecvBuffer.clear();


        size_t prev_size = m_pRecvBuffer.size();
        m_pRecvBuffer.resize(m_pRecvBuffer.size() + nLength);
		memcpy(&m_pRecvBuffer[prev_size], apBuffer, nLength);
        ParseRecvPacket();
    }
    else
    {
		DBG_LOG_ERROR("Error on receiving (%d)", nErrorCode);
    }
}

/*
* Packet Structure
* | Head | SPIN | Length | CMD1 | CMD2 |  Data  | Checksum | Tail |
* |  2b  |  1b  |   2b   |  1b  |  1b  | 0~245b |    2b    |  2b  |
* Head: 0x025B
* Tail: 0x5D03
* Length: 10 + Data Length (range 10 ~ 65535)
*/
void
CEcatInterface::ParseRecvPacket()
{
	int nPacketLength;
	size_t nRemainPacketIndex = 0;

	auto it = m_pRecvBuffer.begin();
	while ((it = std::search(it, m_pRecvBuffer.end(), pbHead.begin(), pbHead.end())) != m_pRecvBuffer.end())
	{
		size_t idx = std::distance(m_pRecvBuffer.begin(), it++);
		if (m_pRecvBuffer.size() >= idx + (size_t)PACKET_MIN_LENGTH)
		{
			nPacketLength = m_pRecvBuffer[idx + 3] * 256 + m_pRecvBuffer[idx + 4] + (int)(PACKET_MIN_LENGTH-1);
			if (m_pRecvBuffer.size() >= idx + nPacketLength)
			{
				if (m_pRecvBuffer[idx + nPacketLength - 2] == 0x5D && m_pRecvBuffer[idx + nPacketLength - 1] == 0x03)
				{
					try 
					{
						BYTEARRAY packet = { m_pRecvBuffer.begin() + idx, m_pRecvBuffer.begin() + idx + nPacketLength };
						InterpretPacket(packet);
					}
					catch (std::bad_array_new_length&)
					{

					}
					nRemainPacketIndex = idx + nPacketLength;
				}
			}
		}
	}

	if (m_pRecvBuffer.size() >= nRemainPacketIndex)
	{
		m_pRecvBuffer = { m_pRecvBuffer.begin() + nRemainPacketIndex, m_pRecvBuffer.end() };
	}
}

void
CEcatInterface::InterpretPacket (const BYTEARRAY aPacket)
{
    m_nSpinId = aPacket[2];
	BYTE command1 = aPacket[5];
	BYTE command2 = aPacket[6];
	BYTEARRAY data = { aPacket.begin() + 6, aPacket.end() - 4 };
	BYTEARRAY crc_array = { aPacket.end() - 4, aPacket.end() - 2 };
	uint16_t crc_recv = (uint16_t) (crc_array[0] * 256 + crc_array[1]); 
	(void) crc_recv; // temp: to suppress unused variable warning
	BYTEARRAY temp{ aPacket.begin() + 2, aPacket.end() - 4 };
	uint16_t crc_calc = m_crc16.calculate(temp); 
	(void) crc_calc; // temp: to suppress unused variable warning
	
	BYTEARRAY feedback;
	BYTEARRAY feedback2;
	//BYTE status;
	//int temp_stat = 0;

	switch (command1)
	{
	case HEAD_COMMON:
		switch (command2)
		{
		case HEAD_GET_ECAT_STATE:
			feedback.push_back(m_stEcatState.uMaster);
			feedback.push_back(m_stEcatState.uSlave);
			feedback.push_back(m_stEcatState.uDomain);
			feedback2 = ConvertUShortToByteArray(m_stEcatState.usNoOfSlaves);
			feedback.insert(feedback.end(), feedback2.begin(), feedback2.end());
			SendPacket(HEAD_GET_ECAT_STATE, feedback);
			break;
		default:
			m_stCurCommand.eCmd = HEAD_NO_CMD2;
			break;
		}
		break;
	case HEAD_ECAT:
		switch (command2)
		{
		
		}
		break;
	default:
		break;
	}

	if (m_nSpinId < 0xFF)
		m_nSpinId++;
	else
		m_nSpinId = 0;
}

BYTEARRAY
CEcatInterface::MakePacket(BYTE anCmd, BYTE anSubCmd, BYTEARRAY apData)
{
	BYTEARRAY packet = {0x02, 0x5B};
	try
	{
		// Only size of Cmd, SubCmd, and total length of data are counted in Packet Length
		size_t nPacketLength = apData.size() + 2;
		packet.push_back(m_nSpinId);
		packet.push_back((uint8_t)((nPacketLength & 0xFF00) >> 8));
		packet.push_back((uint8_t)(nPacketLength & 0x00FF));
		packet.push_back(anCmd);
		packet.push_back(anSubCmd);
		for (size_t i = 0; i < apData.size(); i++)
		{
			packet.push_back(apData[i]);
		}
		
		// STX and ETX are omitted when calculating for the CRC
		BYTEARRAY temp{ packet.begin() + 2, packet.end() };	
		uint16_t crc = m_crc16.calculate(temp);
		packet.push_back((uint8_t)((crc & 0xFF00) >> 8));
		packet.push_back((uint8_t)(crc & 0x00FF));
		packet.push_back(0x5D);
		packet.push_back(0x03);
	}
	catch (std::bad_array_new_length&)
	{

	}

	return packet;
}

BOOL
CEcatInterface::SendPacket(BYTE anCommand, BYTEARRAY apData, BYTE aeCmdType)
{
    BOOL bRet = TRUE;
	BYTEARRAY packet = MakePacket(aeCmdType, anCommand, apData);
	if ( m_pSocket )
	{
		m_pSocket->SendToClients(&packet[0], (uint32_t)(packet.size() & 0xFFFFFFFF));
		if (m_nSpinId < 0xFF)
			m_nSpinId++;
		else
			m_nSpinId = 0;
	}

	return bRet;
}

BOOL
CEcatInterface::SendPacket(BYTE anCommand, BYTE aeCmdType)
{
    BYTEARRAY data{};
    return SendPacket(anCommand, data, aeCmdType);
}


void
CEcatInterface::SendAck(eResponse anResponse)
{
    BYTEARRAY data = {(BYTE) anResponse};
    //SendPacket(HEAD_RESPONSE, data);
}

void 
CEcatInterface::UpdateEcatState(UINT8 auMasterState, UINT8 auSlaveState, UINT8 auDomainState, UINT16 ausNoOfSlaves)
{
	m_stEcatState.uMaster = auMasterState;
	m_stEcatState.uSlave = auSlaveState;
	m_stEcatState.uDomain = auDomainState;
	m_stEcatState.usNoOfSlaves = ausNoOfSlaves;
}

void
CEcatInterface::UpdateCurState(int anPos, unsigned short asVel, unsigned short asAcc, int anJerk, bool abSwitched, bool abReady, bool abOpEnabled, bool abFault)
{
	m_stCurState.position = anPos;
	m_stCurState.velocity = asVel;
	m_stCurState.acceleration = asAcc;
	m_stCurState.jerk = anJerk;
	m_stCurState.switched = abSwitched;
	m_stCurState.ready	= abReady;
	m_stCurState.openabled = abOpEnabled;
	m_stCurState.fault = abFault;
}

void
CEcatInterface::OnSocketSend(PVOID apSocket, PVOID apnError, PVOID apBuffer, PVOID apnLength)
{

}


void
CEcatInterface::OnSocketOpen(PVOID apSocket, PVOID apnError, PVOID apBuffer, PVOID apnLength)
{

}

void
CEcatInterface::OnSocketClose(PVOID apSocket, PVOID apnError, PVOID apBuffer, PVOID apnLength)
{

}

void
CEcatInterface::OnSocketClientDisconnect(PVOID apSocket, PVOID apnError, PVOID apBuffer, PVOID apnLength)
{
	//m_stCurCommand.eCmd = (eHeader)HEAD_SERVO_POWER_OFF;
	// ResetCmdData();
}


void
CEcatInterface::OnSocketAccept(PVOID apSocket, PVOID apnError, PVOID apBuffer, PVOID apnLength)
{

}

void 
CEcatInterface::ResetCmdData(	)
{
	ST_CUR_CMD stemptycmd;
	m_stCurCommand = stemptycmd;
}