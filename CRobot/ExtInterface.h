/*****************************************************************************
*	Name: ExtInterface.cpp
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Header for the External Interface Class
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/

#ifndef __EXTERNAL_INTERFACE__
#define __EXTERNAL_INTERFACE__

#include "Defines.h"
#include "Socket.h"
#include "CRC16.h"
#include "Robot.h"

typedef std::function<void(PVOID, PVOID, PVOID, PVOID)> CALLBACK_FN;

/*
* we need to forward define robot here,
* CExtInterface is also forward defined in Robot.h
*/ 
class CRobot;

#define EXT_IFACE_PORT 7420
#define MIN_PACKET_LENGTH (11)

#define PACKET_STX_H 0x02
#define PACKET_STX_L 0x5B

#define PACKET_ETX_H 0x5D
#define PACKET_ETX_L 0x03


#define PRECISION_FACTOR (1000)
#define ALL_AXIS (INT)(-1)

typedef enum
{
	CMD_NOTHING = 0x00,
	CMD_ROBOT	= 0x82, // 'R'
	CMD_AXIS	= 0x65,	// 'A'
	CMD_ECAT_MASTER	= 0x77, // 'M'
	CMD_ECAT_SLAVE	= 0x83, // 'S'
	CMD_BINARY_RESPONSE = 0x66, // 'B'
} eCommand;

typedef enum
{
	SUBCMD_NOTHING = 0x00,
	SUBCMD_GET_METADATA,
	SUBCMD_GET_STATE,
	SUBCMD_SET_POS = 0x50,
	SUBCMD_GET_POS,
	SUBCMD_SET_VEL,
	SUBCMD_GET_VEL,
	SUBCMD_SET_ACC,
	SUBCMD_GET_ACC,
	SUBCMD_SET_TOR,
	SUBCMD_GET_TOR,
	SUB_CMD_ACK = 0xFE,
	SUB_CMD_NACK = 0xFF,
	
}eSubCommand;


class CExtInterface
{
public:
	CExtInterface();
	~CExtInterface();

	BOOL	Init							(CRobot* apRobot, INT32 anPort=(INT32)EXT_IFACE_PORT);
	BOOL	SendPacket						(BYTE anCmd, BYTE anSubCmd);
	BOOL	SendPacket						(BYTE anCmd, BYTE anSubCmd, BYTEARRAY apData);
	BOOL	DeInit();

private:
	CTcpServer		*m_pSocket;
	BYTE			m_nSpinId;
	BYTEARRAY		m_pRecvBuffer;
	CRC16_ARC		m_cCRC;
	CRobot*			m_pRobot;


public:
	void			RegisterAxis				(CAxis*);
	void			UpdateAxisState				(INT, CAxis*);
	void			UpdateEcatMetadata			(UINT16 ausSlaveNo, UINT8 abtMaster, UINT8 abtSlave, UINT8 abtDomain);
	void			RegisterCallbackAxisCmd		(CALLBACK_FN afnCallback);

private:
	BOOL							m_bInRTContext;
	ST_ECAT_METADATA				m_stEcatMetadata;
	std::vector< ST_AXIS_METADATA>	m_vecStAxisMetadata;
	std::vector<ST_AXIS_STATE>		m_vecStAxisState;
	CALLBACK_FN						m_pCallbackAxisCommand;

private:
	void        OnSocketOpen				(PVOID, PVOID, PVOID, PVOID);
	void        OnSocketSend				(PVOID, PVOID, PVOID, PVOID);
	void        OnSocketClose				(PVOID, PVOID, PVOID, PVOID);
	void        OnSocketClientDisconnect	(PVOID, PVOID, PVOID, PVOID);
	void        OnSocketReceive				(PVOID, PVOID, PVOID, PVOID);
	void        OnSocketAccept				(PVOID, PVOID, PVOID, PVOID);


private:
	BYTEARRAY	MakePacket					(BYTE anCmd, BYTE anSubCmd, BYTEARRAY apData);
	void		ParseRecvPacket				(	);
	void		InterpretPacket				(const BYTEARRAY);
	void		InterpretCmdRobot			(BYTE anSubCmd, BYTEARRAY apData);
	void		InterpretCmdAxis			(BYTE anSubCmd, BYTEARRAY apData);
	void		InterpretCmdEcatMaster		(BYTE anSubCmd, BYTEARRAY apData);
	void		InterpretCmdEcatSlave		(BYTE anSubCmd, BYTEARRAY apData);


private:
	BOOL		SendMetadataRobot			(	);
	BOOL		SendMetadataAxis			(INT anAxis );
	BOOL		SendMetadataEcatMaster		(	);


	BOOL		SendAck();
	BOOL		SendNack();

};



#endif //__EXTERNAL_INTERFACE__