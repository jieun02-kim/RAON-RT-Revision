/*****************************************************************************
*	Name: EcatCommon.h
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Common definitions and structures for EtherCAT related stuff
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
#ifndef _ECAT_COMMON_H_
#define _ECAT_COMMON_H_

#include "Defines.h"
#include "ecrt.h"


/* CONTROL WORD */
#define CIA402_SHUTDOWN					(UINT16)0x0006
#define	CIA402_SWITCH_ON				(UINT16)0x0007
#define	CIA402_SWITCH_ON_EN_OP			(UINT16)0x000F
#define	CIA402_DISABLE_VOLTAGE			(UINT16)0x0000
#define	CIA402_QUICK_STOP				(UINT16)0x0002
#define	CIA402_DISABLE_OPERATION		(UINT16)0x0007
#define	CIA402_ENABLE_OPERATION			(UINT16)0x000F
#define	CIA402_FAULT_RESET				(UINT16)0x0080

/* STATUS WORD */
#define	CIA402_NOT_READY_TO_SWITCH_ON	(UINT16)0x0000
#define	CIA402_SWITCH_ON_DISABLED		(UINT16)0x0040
#define	CIA402_READY_TO_SWITCH_ON		(UINT16)0x0021
#define	CIA402_SWITCH_ON_ENABLED		(UINT16)0x0023
#define	CIA402_OPERATION_ENABLED		(UINT16)0x0027
#define	CIA402_QUICK_STOP_ACTIVE		(UINT16)0x0007
#define	CIA402_FAULT_REACTION_ACTIVE	(UINT16)0x000F
#define	CIA402_FAULT					(UINT16)0x0008

/* DRIVE MODES */
#define CIA402_PROFILE_POSITION			(INT8)0x01
#define CIA402_PROFILE_VELOCITY			(INT8)0x03
#define CIA402_PROFILE_TORQUE			(INT8)0x04
#define CIA402_HOMING					(INT8)0x06
#define CIA402_INTERPOLATED_POSITION	(INT8)0x07
#define CIA402_CYCLIC_POSITION			(INT8)0x08
#define CIA402_CYCLIC_VELOCITY			(INT8)0x09
#define CIA402_CYCLIC_TORQUE			(INT8)0x0A


#define MAX_SYNC_PDO	100
#define MAX_ECAT_SLAVES	65536

inline TSTRING	GetCIA402DriveMode		(int anDriveMode);
inline INT8		ValidateCIA402DriveMode	(int anDriveMode);

/* EtherCAT Specific Error Codes */
#define EMSCONF	1	/* Master Slave Configuration */
#define EPDCONF	2	/* PDO Configuration */	 
#define EDCONF	3	/* DC Configuration */


#define DEFAULT_VENDORID -1
#define DEFAULT_PRODUCTCODE -1

/* IgH EtherCAT Master API (refer to ecrt.h) */
typedef	ec_master_t				ECAT_MASTER,				*PECAT_MASTER;
typedef	ec_domain_t 			ECAT_DOMAIN,				*PECAT_DOMAIN;
typedef ec_slave_config_t		ECAT_SLAVE_CONFIG,			*PECAT_SLAVE_CONFIG;

typedef	uint8_t					ECAT_DOMAIN_PD,				*PECAT_DOMAIN_PD;
typedef	ec_master_state_t		ECAT_MASTER_STATE,			*PECAT_MASTER_STATE;
typedef ec_domain_state_t		ECAT_DOMAIN_STATE,			*PECAT_DOMAIN_STATE;
typedef ec_slave_config_state_t	ECAT_SLAVE_CONFIG_STATE,	*PECAT_SLAVE_CONFIG_STATE;
typedef ec_sync_info_t			ECAT_SYNC_INFO,				*PECAT_SYNC_INFO;

/* Master and Slave States */
typedef enum 
{
	eNONE		=	0x00,	/* Not Switched-On */ 
	eINIT		=	0x01,	/* Mailbox (X), Process Data (X)  */
	ePRE_OP		=	0x02,	/* Mailbox (O), Process Data (X)  */
	eSAFE_OP	=	0x04,	/* Mailbox (O), Process Data (limited) */
	eOP			=	0x08	/* Mailbox (O), Process Data (O) */
} ECAT_STATE_MACH;

/* Domain */
typedef enum 
{
	eWC_NO_EXCHANGED	=	0,	/* No registered process data were exchanged */
	eWC_INCOMPLETE,				/* Some of the registered process data were  exchanged */
	eWC_COMPLETE				/* All registered process data were exchanged */
} ECAT_WC_STATE;

typedef unsigned int ECAT_WC;

/* Device Type */
typedef enum 
{
	eUNKNOWN	=	0,
	eDIGITAL_IN,
	eDIGITAL_OUT,
	eDIGITAL_IN_OUT,
	eANALOG_IN,
	eANALOG_OUT,
	eANALOG_IN_OUT,
	eTERMINAL,
	eJUNCTION,
	eCIA402,
	eFTSensor,
	eKistarHand,
} ECAT_DEVICE_TYPE;

/* Slave Type */
typedef enum 
{
	eNO_IO	=	0,
	eIN_ONLY,
	eOUT_ONLY,
	eIN_OUT
} ECAT_SLAVE_TYPE;

typedef enum 
{
	eInput	=	0,
	eOutput
}ECAT_COMM_DIR;


typedef struct ECAT_MASTER_DESC
{
	PECAT_MASTER	pEcatMaster;
	PECAT_DOMAIN	pEcatDomainIn;
	PECAT_DOMAIN	pEcatDomainOut;
	PECAT_DOMAIN_PD	pEcatDomainPdIn;
	PECAT_DOMAIN_PD	pEcatDomainPdOut;

	ECAT_MASTER_DESC()
	{
		pEcatMaster			=	NULL;
		pEcatDomainIn		=	NULL;
		pEcatDomainOut		=	NULL;
		pEcatDomainPdIn		=	NULL;
		pEcatDomainPdOut	=	NULL;
	}

} stEcatMasterDesc;

typedef	struct ECAT_SLAVE_INFO
{
	UINT32					unVendorID;			// Vendor ID
	UINT32					unProdCode;			// Product Code
	UINT16					usAlias;			// Alias
	UINT16					usPosition;			// Position
	
	struct DC_INFO
	{
		BOOL				bIsSupported;		// DC support		--> Refer to slave manual
		UINT16				usActivateWord;		// DC Activate Word --> Refer to slave manual
		UINT32				unCycleTime;		// DC Shift Time 	--> Refer to slave manual
		INT32				nShiftTime;			// DC Shift Time 	--> Refer to slave manual
		DC_INFO()
		{
			bIsSupported	=	FALSE;
			usActivateWord	=	0;
			unCycleTime		=	0;
			nShiftTime		=	0;
		}
	}stEcatDCInfo;

	ECAT_SLAVE_TYPE			eSlaveType;			// Slave Type
	PECAT_SYNC_INFO			pEcatSyncInfo;		// Sync Info	
	

	ECAT_SLAVE_INFO()
	{
		unVendorID = 0;
		unProdCode = 0;
		usAlias = 0;
		usPosition = 0;
		eSlaveType = eNO_IO;
		pEcatSyncInfo = NULL;
	}

}stEcatSlaveInfo;

inline TSTRING 
GetCIA402DriveMode(int anDriveMode)
{
	switch (anDriveMode)
	{
	case CIA402_PROFILE_POSITION:
		return "PROFILE POSITION";
		break;
	case CIA402_PROFILE_VELOCITY:
		return "PROFILE VELOCITY";
		break;
	case CIA402_PROFILE_TORQUE:
		return "PROFILE TORQUE";
		break;
	case CIA402_HOMING:
		return "HOMING";
		break;
	case CIA402_INTERPOLATED_POSITION:
		return "INTERPOLATED POSITION";
		break;
	case CIA402_CYCLIC_POSITION:
		return "CYCLIC POSITION";
		break;
	case CIA402_CYCLIC_VELOCITY:
		return "CYCLIC VELOCITY";
		break;
	case CIA402_CYCLIC_TORQUE:
		return "CYCLIC TORQUE";
		break;
	default:
		break;
	}
	return "";
}

inline INT8
ValidateCIA402DriveMode(int anDriveMode)
{
	switch (anDriveMode)
	{
	case 1:
		return CIA402_PROFILE_POSITION;
		break;
	case 3:
		return CIA402_PROFILE_VELOCITY;
		break;
	case 4:
		return CIA402_PROFILE_TORQUE;
		break;
	case 6:
		return CIA402_HOMING;
		break;
	case 7:
		return CIA402_INTERPOLATED_POSITION;
		break;
	case 8:
		return CIA402_CYCLIC_POSITION;
		break;
	case 9:
		return CIA402_CYCLIC_VELOCITY;
		break;
	case 10:
		return CIA402_CYCLIC_TORQUE;
		break;
	default:
		break;
	}
	/* default is Profile Position Mode */
	return CIA402_PROFILE_POSITION;
}


#endif //_ECAT_COMMON_H_