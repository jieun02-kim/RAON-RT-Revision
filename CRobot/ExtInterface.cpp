/*****************************************************************************
*	Name: ExtInterface.cpp
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Implementation of the External Interface class
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
#include "ExtInterface.h"
#include <algorithm>

static const BYTEARRAY pbHead = { 0x02, 0x5B };

CExtInterface::CExtInterface()
{
	m_pSocket = NULL;
	m_pRobot = NULL;
	m_nSpinId = 0;
	m_cCRC = CRC16_ARC();
	m_pRecvBuffer.clear();
	m_bInRTContext = FALSE;
	m_vecStAxisMetadata.clear();
}

CExtInterface::~CExtInterface()
{

}

BOOL
CExtInterface::Init(CRobot* apRobot, INT32 anPort)
{
	DeInit();

	m_pRobot = apRobot;
	if (NULL == m_pRobot)
	{
		DBG_LOG_ERROR("(%s) Cannot Initialize! Create Robot Instance first.", "CExtInterface");
		return FALSE;
	}
	
	m_pSocket = new CTcpServer();

	if (m_pSocket)
	{
		m_pSocket->RegisterCallbackStart(std::bind(&CExtInterface::OnSocketOpen, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
		m_pSocket->RegisterCallbackReceive(std::bind(&CExtInterface::OnSocketReceive, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
		m_pSocket->RegisterCallbackSend(std::bind(&CExtInterface::OnSocketSend, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
		m_pSocket->RegisterCallbackStop(std::bind(&CExtInterface::OnSocketClose, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
		m_pSocket->RegisterCallbackAccept(std::bind(&CExtInterface::OnSocketAccept, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
		m_pSocket->RegisterCallbackClientDisconnect(std::bind(&CExtInterface::OnSocketClientDisconnect, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
	}

	if (FALSE == m_pSocket->Start(anPort))
	{
		DeInit();
		return FALSE;
	}

	DBG_LOG_INFO("(%s) External Interface Initialized in Port %d.", "CExtInterface", anPort);
	return TRUE;
}

BOOL
CExtInterface::DeInit()
{
	if (m_pSocket)
	{
		m_pSocket->Stop();
		delete m_pSocket;
		m_pSocket = NULL;
	}

	m_vecStAxisMetadata.clear();

	return TRUE;
}

void
CExtInterface::RegisterAxis(CAxis* apAxis)
{
	ST_AXIS_METADATA stMetadata;

	stMetadata.strName = apAxis->GetName();
	stMetadata.btAxisType = (UINT8)apAxis->GetAxisType();
	stMetadata.btCommType = (UINT8)apAxis->GetCommType();

	ST_AXIS_LIMITS stLimits = apAxis->GetAxisLimits();

	if ((UINT8)eAxisRevolute == stMetadata.btAxisType)
	{
		stMetadata.nPosLimitU = (INT32)(ConvertRad2Deg(stLimits.stPos.dUpper) * PRECISION_FACTOR);
		stMetadata.nPosLimitL = (INT32)(ConvertRad2Deg(stLimits.stPos.dLower) * PRECISION_FACTOR);	
	}
	else
	{
		stMetadata.nPosLimitU = (INT32)(stLimits.stPos.dUpper * PRECISION_FACTOR);
		stMetadata.nPosLimitL = (INT32)(stLimits.stPos.dLower * PRECISION_FACTOR);
	}

	/* todo: consider torque and deceleration later */		
	stMetadata.nVelLimitU = (INT32)(stLimits.stVel.dUpper * PRECISION_FACTOR);
	stMetadata.nVelLimitL = (INT32)(stLimits.stVel.dLower * PRECISION_FACTOR);

	stMetadata.nAccLimitU = (INT32)(stLimits.stAcc.dUpper * PRECISION_FACTOR);
	stMetadata.nAccLimitL = (INT32)(stLimits.stAcc.dLower * PRECISION_FACTOR);

	m_vecStAxisMetadata.push_back(stMetadata);

	ST_AXIS_STATE stAxisState;

	stAxisState.btDriveMode = (BYTE)apAxis->GetDriveMode();
	stAxisState.btStatus = (BYTE)apAxis->GetState();
	stAxisState.dCurVel = (double)1.0;
	stAxisState.dCurPos = (double)1.0;
	stAxisState.dCurAcc = (double)1.0;
	stAxisState.dCurTor = (double)1.0;

	m_vecStAxisState.push_back(stAxisState);
}

void
CExtInterface::UpdateAxisState(INT anAxisID, CAxis* apAxis)
{

	if (m_vecStAxisState.empty() || anAxisID > (INT)m_vecStAxisState.size())
		return;

	m_vecStAxisState[anAxisID].btDriveMode = (BYTE)apAxis->GetDriveMode();
	m_vecStAxisState[anAxisID].btStatus = (BYTE)apAxis->GetState();
	m_vecStAxisState[anAxisID].dCurPos = (BYTE)apAxis->GetCurrentPos();
	m_vecStAxisState[anAxisID].dCurVel = (BYTE)apAxis->GetCurrentPos();
	m_vecStAxisState[anAxisID].dCurAcc = (BYTE)apAxis->GetCurrentPos();
}

void
CExtInterface::UpdateEcatMetadata(UINT16 ausSlaveNo, UINT8 abtMaster, UINT8 abtSlave, UINT8 abtDomain)
{
	m_stEcatMetadata.usSlaveNo = ausSlaveNo;
	m_stEcatMetadata.stEcatStates.btMaster = abtMaster;
	m_stEcatMetadata.stEcatStates.btSlave = abtSlave;
	m_stEcatMetadata.stEcatStates.btDomain = abtDomain;

	m_bInRTContext = TRUE;
}

void
CExtInterface::RegisterCallbackAxisCmd(CALLBACK_FN afnCallback)
{
	if (NULL == m_pCallbackAxisCommand)
	{
		m_pCallbackAxisCommand = std::move(afnCallback);
	}
}


BYTEARRAY
CExtInterface::MakePacket(BYTE anCmd, BYTE anSubCmd, BYTEARRAY apData)
{
	/* STX */
	BYTEARRAY vPacket = { 0x02, 0x5B };

	try
	{
		/* Only size of Cmd, SubCmd, and total length of data are counted in Packet Length */
		size_t nPacketLength = apData.size() + 2;
		vPacket.push_back(m_nSpinId);
		vPacket.push_back((uint8_t)((nPacketLength & 0xFF00) >> 8));
		vPacket.push_back((uint8_t)(nPacketLength & 0x00FF));
		vPacket.push_back(anCmd);
		vPacket.push_back(anSubCmd);
		for (size_t i = 0; i < apData.size(); i++)
		{
			vPacket.push_back(apData[i]);
		}

		/* STX and ETX are omitted when calculating for the CRC */
		BYTEARRAY temp{ vPacket.begin() + 2, vPacket.end() };
		uint16_t crc = m_cCRC.calculate(temp);
		vPacket.push_back((uint8_t)((crc & 0xFF00) >> 8));
		vPacket.push_back((uint8_t)(crc & 0x00FF));
		
		/* ETX */
		vPacket.push_back(0x5D);
		vPacket.push_back(0x03);
	}
	catch (std::bad_array_new_length&)
	{
		vPacket.clear();
	}

	return vPacket;
}

BOOL
CExtInterface::SendPacket(BYTE anCmd, BYTE anSubCmd)
{
	BYTEARRAY vData{};
	return SendPacket(anCmd, anSubCmd, vData);
}

BOOL 
CExtInterface::SendPacket(BYTE anCmd, BYTE anSubCmd, BYTEARRAY apData)
{
	BYTEARRAY vPacket = MakePacket(anCmd, anSubCmd, apData);

	if (m_pSocket)
	{
		m_pSocket->SendToClients(&vPacket[0], (UINT32)(vPacket.size() & 0xFFFFFFFF));
		if (m_nSpinId < 0xFF)
			m_nSpinId++;
		else
			m_nSpinId = 0;

		return TRUE;
	}
	
	return FALSE;
}

void
CExtInterface::OnSocketOpen(PVOID apSocket, PVOID apnError, PVOID apBuffer, PVOID apnLength)
{

}

void
CExtInterface::OnSocketSend(PVOID apSocket, PVOID apnError, PVOID apBuffer, PVOID apnLength)
{

}

void
CExtInterface::OnSocketClose(PVOID apSocket, PVOID apnError, PVOID apBuffer, PVOID apnLength)
{

}

void
CExtInterface::OnSocketClientDisconnect(PVOID apSocket, PVOID apnError, PVOID apBuffer, PVOID apnLength)
{

}

void
CExtInterface::OnSocketReceive(PVOID apSocket, PVOID apnError, PVOID apBuffer, PVOID apnLength)
{
	int nErrorCode = *(int*)apnError;
	int nLength = *(int*)apnLength;

	if (nErrorCode == 0)
	{
		if (m_pRecvBuffer.size() >= MAX_RECV_BUFF_LEN)
			m_pRecvBuffer.clear();


		size_t prev_size = m_pRecvBuffer.size();
		m_pRecvBuffer.resize(m_pRecvBuffer.size() + nLength);
		memcpy(&m_pRecvBuffer[prev_size], apBuffer, nLength);
		ParseRecvPacket();
	}
	else
	{
		m_pRecvBuffer.clear();
		DBG_LOG_ERROR("Error on receiving (%d)", nErrorCode);
	}
}

void
CExtInterface::OnSocketAccept(PVOID apSocket, PVOID apnError, PVOID apBuffer, PVOID apnLength)
{
	CTcpClient pSocket = *(CTcpClient*)apSocket;
	DBG_LOG_INFO("(%s) Client Connected: IP:%s, Port:%d", "CExtInterface", pSocket.GetIPAddress().c_str(), pSocket.GetPortNo());
	
	SendMetadataRobot();
	SendMetadataEcatMaster();
	SendMetadataAxis(ALL_AXIS);
}

void
CExtInterface::ParseRecvPacket()
{
	int nPacketLength;
	size_t nRemainPacketIndex = 0;

	auto it = m_pRecvBuffer.begin();
	while ((it = std::search(it, m_pRecvBuffer.end(), pbHead.begin(), pbHead.end())) != m_pRecvBuffer.end())
	{
		size_t idx = std::distance(m_pRecvBuffer.begin(), it++);
		if (m_pRecvBuffer.size() >= idx + (size_t)MIN_PACKET_LENGTH)
		{
			nPacketLength = m_pRecvBuffer[idx + 3] * 256 + m_pRecvBuffer[idx + 4] + (int)(MIN_PACKET_LENGTH - 2);

			if (m_pRecvBuffer.size() >= idx + nPacketLength)
			{
				if (m_pRecvBuffer[idx + nPacketLength - 2] == (BYTE)PACKET_ETX_H && m_pRecvBuffer[idx + nPacketLength - 1] == (BYTE)PACKET_ETX_L)
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
CExtInterface::InterpretPacket(const BYTEARRAY aPacket)
{
	BYTE btCmd = aPacket[5];
	BYTE btSubCmd = aPacket[6];

	BYTEARRAY vData;
	size_t szPacketLen = aPacket.size();
	if (MIN_PACKET_LENGTH > szPacketLen)
		return;

	vData = { aPacket.begin() + 7, aPacket.end() - 4 };

	switch (btCmd)
	{
	case CMD_ROBOT:
		InterpretCmdRobot(btSubCmd, vData);
		break;

	case CMD_AXIS:
		InterpretCmdAxis(btSubCmd, vData);
		break;

	case CMD_ECAT_MASTER:
		InterpretCmdEcatMaster(btSubCmd, vData);
		break;

	case CMD_ECAT_SLAVE:
		InterpretCmdEcatSlave(btSubCmd, vData);
		break;

	case CMD_NOTHING:
	default:
		break;
	}

	if (m_nSpinId < 0xFF)
		m_nSpinId++;
	else
		m_nSpinId = 0;
}

void
CExtInterface::InterpretCmdRobot(BYTE anSubCmd, BYTEARRAY apData)
{
	BYTE nSubCmd = anSubCmd;

	switch (nSubCmd)
	{
	case SUBCMD_GET_METADATA:
		SendMetadataRobot();
		break;

	case SUBCMD_GET_STATE:
		break;

	case SUBCMD_NOTHING:
	default:
		break;
	}
}

void
CExtInterface::InterpretCmdAxis(BYTE anSubCmd, BYTEARRAY apData)
{
	BYTE nSubCmd = anSubCmd;

	
	INT nAxisNo = apData[0];
	BYTEARRAY vData{};
	VEC_AXIS_CMD vAxisCmd{};

	int	nValue = 0;
	double dValue = 0.;

	try
	{
		switch (nSubCmd)
		{
		case SUBCMD_GET_METADATA:
			SendMetadataAxis(nAxisNo);
			break;

		case SUBCMD_GET_STATE:
			if (nAxisNo == ALL_AXIS)
			{
			}
			else
			{
		
		
		
			}
		case SUBCMD_SET_POS:
			if (nAxisNo == ALL_AXIS)
			{
			}
			else
			{
				vData = { apData.begin() + 1, apData.end() };
				nValue = ConvertByteArrayToIntBE(vData);
				dValue = ConvertDeg2Rad((double)nValue / PRECISION_FACTOR);
				ST_AXIS_CMD stAxisCmd;
				stAxisCmd.nAxisNo = nAxisNo;
				stAxisCmd.nCmd = (int)SUBCMD_SET_POS;
				stAxisCmd.dValue = dValue;
				vAxisCmd.push_back(stAxisCmd);

				if (NULL != m_pCallbackAxisCommand)
					m_pCallbackAxisCommand(&vAxisCmd, NULL, NULL, NULL);

				SendAck();
			}
			break;

		case SUBCMD_NOTHING:
		default:
			break;
		}
	}
	catch (std::bad_array_new_length&)
	{
		return;
	}
}

void
CExtInterface::InterpretCmdEcatMaster(BYTE anSubCmd, BYTEARRAY apData)
{
	BYTE nSubCmd = anSubCmd;

	switch (nSubCmd)
	{
	case SUBCMD_GET_METADATA:
		SendMetadataEcatMaster();
		break;

	case SUBCMD_GET_STATE:
		break;

	case SUBCMD_NOTHING:
	default:
		break;
	}
}

void
CExtInterface::InterpretCmdEcatSlave(BYTE anSubCmd, BYTEARRAY apData)
{
	BYTE nSubCmd = anSubCmd;

	switch (nSubCmd)
	{
	case SUBCMD_GET_METADATA:
		break;

	case SUBCMD_GET_STATE:
		break;

	case SUBCMD_NOTHING:
	default:
		break;
	}
}

BOOL
CExtInterface::SendMetadataRobot()
{
	BYTEARRAY vData;
	
	try 
	{
		vData.clear();
		/* Simulation Mode */
		vData.push_back((BYTE)m_pRobot->IsSim());
	
		/* Total Number of Axes */
		ExtendByteArray(vData, ConvertU16ToByteArrayBE((UINT16)m_pRobot->GetTotalAxis()));
	
		/* EtherCAT Enabled */
		vData.push_back((BYTE)m_pRobot->IsEcatEnabled());

		/* Robot Name */
		/* maximum of 255 characters for one byte name length */
		size_t szNameLen = m_pRobot->GetName().length();
		if (szNameLen > 255) szNameLen = 255;

		vData.push_back((BYTE)szNameLen);
		for (int nCnt = 0; nCnt < (int)szNameLen; nCnt++)
		{
			vData.push_back((BYTE)(m_pRobot->GetName().at(nCnt)));
		}
	}
	catch (std::bad_array_new_length&)
	{
		return FALSE;
	}
	
	return SendPacket((BYTE)CMD_ROBOT, (BYTE)SUBCMD_GET_METADATA, vData);
}

BOOL
CExtInterface::SendMetadataAxis(INT anAxis)
{
	BYTEARRAY vData{};
	
	try
	{
		if ((INT)m_vecStAxisMetadata.size() < anAxis || TRUE == m_vecStAxisMetadata.empty())
		{
			return FALSE;
		}
		else if (ALL_AXIS == anAxis)
		{
			vData.clear();
			vData.push_back(ConvertS8ToByte((INT8)ALL_AXIS));
			
			for (int nCnt = 0; nCnt < (int)m_vecStAxisMetadata.size(); nCnt++)
			{
				vData.push_back((BYTE)(UINT8)nCnt);

				ST_AXIS_METADATA stMetadata = m_vecStAxisMetadata[nCnt];
				/* Axis Types */
				vData.push_back((BYTE)stMetadata.btAxisType);
				vData.push_back((BYTE)stMetadata.btCommType);

				/* Axis Limits */
				ExtendByteArray(vData, ConvertS32ToByteArrayBE(stMetadata.nPosLimitU));
				ExtendByteArray(vData, ConvertS32ToByteArrayBE(stMetadata.nPosLimitL));

				ExtendByteArray(vData, ConvertS32ToByteArrayBE(stMetadata.nVelLimitU));
				ExtendByteArray(vData, ConvertS32ToByteArrayBE(stMetadata.nVelLimitL));

				ExtendByteArray(vData, ConvertS32ToByteArrayBE(stMetadata.nAccLimitU));
				ExtendByteArray(vData, ConvertS32ToByteArrayBE(stMetadata.nAccLimitL));
				/* Axis Name */
				/* maximum of 255 characters for one byte name length */
				size_t szNameLen = stMetadata.strName.length();
				if (szNameLen > 255) szNameLen = 255;

				vData.push_back((BYTE)szNameLen);
				for (int nCnt = 0; nCnt < (int)szNameLen; nCnt++)
				{
					vData.push_back((BYTE)(stMetadata.strName.at(nCnt)));
				}
			}
		}
		else
		{
			vData.clear();

			vData.push_back((BYTE)(UINT8)anAxis);
			ST_AXIS_METADATA stMetadata = m_vecStAxisMetadata[anAxis];

			/* Axis Types */
			vData.push_back((BYTE)stMetadata.btAxisType);
			vData.push_back((BYTE)stMetadata.btCommType);

			/* Axis Limits */
			ExtendByteArray(vData, ConvertS32ToByteArrayBE(stMetadata.nPosLimitU));
			ExtendByteArray(vData, ConvertS32ToByteArrayBE(stMetadata.nPosLimitL));

			ExtendByteArray(vData, ConvertS32ToByteArrayBE(stMetadata.nVelLimitU));
			ExtendByteArray(vData, ConvertS32ToByteArrayBE(stMetadata.nVelLimitL));

			ExtendByteArray(vData, ConvertS32ToByteArrayBE(stMetadata.nAccLimitU));
			ExtendByteArray(vData, ConvertS32ToByteArrayBE(stMetadata.nAccLimitL));

			/* Axis Name */
			/* maximum of 255 characters for one byte name length */
			size_t szNameLen = stMetadata.strName.length();
			if (szNameLen > 255) szNameLen = 255;

			vData.push_back((BYTE)szNameLen);
			for (int nCnt = 0; nCnt < (int)szNameLen; nCnt++)
			{
				vData.push_back((BYTE)(stMetadata.strName.at(nCnt)));
			}
		}
	}
	catch (std::bad_array_new_length&)
	{
		return FALSE;
	}
	return SendPacket((BYTE)CMD_AXIS, (BYTE)SUBCMD_GET_METADATA, vData);
}

BOOL
CExtInterface::SendMetadataEcatMaster()
{
	if (NULL == m_pRobot->m_pcEcatMaster)
		return FALSE;

	BYTEARRAY vData;
	try
	{
		UINT8 btMasterState, btSlaveState, btDomainState;
		UINT16 usSlaveNo;

		if (m_bInRTContext)
		{
			btMasterState = m_stEcatMetadata.stEcatStates.btMaster;
			btSlaveState = m_stEcatMetadata.stEcatStates.btSlave;
			btDomainState = m_stEcatMetadata.stEcatStates.btDomain;
			usSlaveNo = m_stEcatMetadata.usSlaveNo;
		}
		else
		{
			btMasterState = m_pRobot->m_pcEcatMaster->GetMasterState();
			btSlaveState = m_pRobot->m_pcEcatMaster->GetSlaveState();
			btDomainState = m_pRobot->m_pcEcatMaster->GetDomainState();
			usSlaveNo = (UINT16)m_pRobot->m_pcEcatMaster->GetSlaveCnt();
		}

		vData.clear();
		/* EtherCAT States */
		vData.push_back((BYTE)btMasterState);
		vData.push_back((BYTE)btSlaveState);
		vData.push_back((BYTE)btDomainState);
		/* Slave Number */
		ExtendByteArray(vData, ConvertU16ToByteArrayBE((UINT16)usSlaveNo));
	}
	catch (std::bad_array_new_length&)
	{
		return FALSE;
	}

	return SendPacket((BYTE)CMD_ECAT_MASTER, (BYTE)SUBCMD_GET_METADATA, vData);
}

BOOL
CExtInterface::SendAck()
{
	return SendPacket((BYTE)CMD_BINARY_RESPONSE, (BYTE)SUB_CMD_ACK);
}

BOOL
CExtInterface::SendNack()
{
	return SendPacket((BYTE)CMD_BINARY_RESPONSE, (BYTE)SUB_CMD_NACK);
}
