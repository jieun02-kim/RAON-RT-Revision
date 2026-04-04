/*****************************************************************************
*	Name: EcatInterface.h
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Header for the CEcatInterface class
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
#ifndef _ECAT_INTERFACE_
#define _ECAT_INTERFACE_

#include <sys/socket.h>
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
#include <algorithm>
#include <functional>

#include "Socket.h"
#include "CRC16.h"
#include "EcatMasterBase.h"

#define DEFAULT_PORT    7421

// STX (2) + SPIN (1) + LENGTH (2) + CMD1 (1) + CMD2 (1) + CRC (2) + ETX (2)
#define PACKET_MIN_LENGTH 11

typedef enum 
{
	HEAD_NO_CMD     			= 0x00,
	HEAD_COMMON					= 0x43, // 'C'
	HEAD_ECAT					= 0x45, // 'E'
} eHeader;

typedef enum
{
	HEAD_NO_CMD2				= 0x00,
	HEAD_GET_ECAT_STATE			= 0x01,
	//HEAD_MOVE_ABS_POS			= 0x02,
	//HEAD_MOVE_ABS_POS_FORCED	= 0x03,
	//HEAD_MOVE_REL_POS			= 0x03,
	//HEAD_MOVE_REL_POS_FORCED	= 0x04,
	//HEAD_HOMING				= 0x05,
	//HEAD_CUR_POSITION			= 0x06,
	//HEAD_SET_PROFILE_VEL		= 0x07,
	//HEAD_GET_PROFILE_VEL		= 0x08,
	//HEAD_SET_PROFILE_ACC		= 0x09,
	//HEAD_GET_PROFILE_ACC		= 0x0A,
	//HEAD_SET_PROFILE_JERK		= 0x0B,
	//HEAD_GET_PROFILE_JERK		= 0x0C,
	//HEAD_HALT_OPERATION		= 0x0D,
	//HEAD_SERVO_POWER_ON		= 0x10,
	//HEAD_SERVO_POWER_OFF		= 0x11,
	//HEAD_GET_SERVOSTATUS		= 0x12,
	//HEAD_FAULT_RESET			= 0x13,
	HEAD_RESPONSE				= 0x23,
} eHeader2;

typedef enum
{
	RESP_ACK_OK		= 0x06,
	RESP_NACK		= 0x07,
	RESP_TIMEOUT	= 0x08
} eResponse;

#pragma pack(push, 1)
typedef struct _ST_CUR_STATE
{
	int				position;
	unsigned short	velocity;
	unsigned short	acceleration;
	unsigned int	jerk;
	bool			switched;
	bool			ready;
	bool			openabled;
	bool			fault;

	_ST_CUR_STATE()
	{
		position		= 0;
		velocity		= 0;
		acceleration	= 0;
		jerk			= 0;
		switched		= false;
		ready			= false;
		openabled		= false;
		fault			= false;
	}
} ST_CUR_STATE;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct _ST_CUR_CMD
{
	INT32			nAlias;
	INT32			nPosition;
	eHeader2        eCmd;
	INT64           nData;

	_ST_CUR_CMD()
	{
		nAlias		= 0;
		nPosition	= 0;
		eCmd		= HEAD_NO_CMD2;
		nData		= 0;
	}
} ST_CUR_CMD;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct _ST_CUR_ECAT_STATE
{
	UINT8			uMaster;
	UINT8			uSlave;
	UINT8			uDomain;
	UINT16			usNoOfSlaves;

	_ST_CUR_ECAT_STATE()
	{
		uMaster = 0;
		uSlave = 0;
		uDomain = 0;
		usNoOfSlaves = 0;
	}
} ST_CUR_ECAT_STATE;
#pragma pack(pop)


class CEcatInterface
{
private:
    CTcpServer				*m_pSocket;
    CEcatMasterBase			*m_pcEcatMaster;   
    BYTE                    m_nSpinId;
    BYTEARRAY			    m_pRecvBuffer;
    CRC16_ARC               m_crc16;
	ST_CUR_ECAT_STATE		m_stEcatState;
    ST_CUR_STATE		    m_stCurState;
    ST_CUR_CMD              m_stCurCommand;

private:
    void        ParseRecvPacket 			(   );
    void        InterpretPacket 			(const BYTEARRAY);
    void        OnSocketOpen    			(PVOID, PVOID, PVOID, PVOID);
    void        OnSocketSend    			(PVOID, PVOID, PVOID, PVOID);
    void        OnSocketClose   			(PVOID, PVOID, PVOID, PVOID);
	void        OnSocketClientDisconnect	(PVOID, PVOID, PVOID, PVOID);
	void        OnSocketReceive				(PVOID, PVOID, PVOID, PVOID);
	void        OnSocketAccept				(PVOID, PVOID, PVOID, PVOID);
    
    BYTEARRAY   MakePacket      (BYTE anCommand1, BYTE anCommand2, BYTEARRAY apData);
    void        SendAck         (eResponse anResponse);


public:
    CEcatInterface();
    virtual ~CEcatInterface();

    BOOL    Init    (CEcatMasterBase* apcEcatMaster = NULL, int anPort=(int)DEFAULT_PORT); // NULL is for testing only
    BOOL    DeInit  (   );

	BOOL    	SendPacket			(BYTE, BYTE aeCmdType=HEAD_COMMON);
	BOOL    	SendPacket			(BYTE, BYTEARRAY, BYTE aeCmdType=HEAD_COMMON);
    BOOL    	IsConnected			(    );
    ST_CUR_CMD  GetCmdData      	(   ){return m_stCurCommand;};
	void  		ResetCmdData      	(   );
	void		UpdateCurState		(int, unsigned short, unsigned short, int, bool, bool, bool, bool);
	void		UpdateEcatState		(UINT8, UINT8, UINT8, UINT16);
    
};

#endif // CEcatInterface