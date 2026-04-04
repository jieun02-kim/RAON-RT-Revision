/*****************************************************************************
*	Name: SlaveCIA402Base.h
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Header for the CSlaveCIA402Base child class.
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
#ifndef _ECAT_SLAVE_CIA_402_BASE_
#define _ECAT_SLAVE_CIA_402_BASE_

#include "EcatSlaveBase.h"



typedef std::function<void(PVOID, PVOID, PVOID, PVOID)> CALLBACK_FN;

/* Servo Status */
typedef enum 
{
	eServoOff = 0x00,
	eServoIdle,
	eServoStopped,
	eServoHoming,
	eServoRunning,
	eServoFault = 0x80,
	eServoDisconnected,
} eSERVO_STATUS;

typedef struct stRawData
{
	INT16 usControlWord;
	INT32 nTargetPos;
	INT32 nTargetVel;
	INT16 nTargetTor;
	INT8  btDriveMode;

	INT16 usStatusWord;
	INT32 nActualPos;
	INT32 nActualVel;
	INT16 nActualTor;
	INT8  btActualDriveMode;

	stRawData()
	{
		usControlWord = 0;
		nTargetPos = 0;
		nTargetVel = 0;
		nTargetTor = 0;
		btDriveMode = 0;
		usStatusWord = 0;
		nActualPos = 0;
		nActualVel = 0;
		nActualTor = 0;
		btActualDriveMode = 0;
	}
}ST_RAW_DATA;

typedef enum
{
	eAbsolute = 0x0,
	eRelative,

} ePOSITION_MODE;

class CSlaveCIA402Base : public CEcatSlaveBase
{
private:
	struct CIA402_OUT //Output 
	{
		UINT16	usControlWord;
		INT32	nTargetPos;
		INT32	nTargetVel;
		INT32	nOffsetVel;
		INT16	sTargetTor;
		INT16	sOffsetTor;
		UINT32	unProfileVel;
		UINT32	unProfileAcc;
		UINT32	unProfileDec;
		UINT32	unQuickStopDec;
		UINT32	unProfileJerk;
		UINT32	unMaxProfileVel;
		UINT32	unMaxProfileAcc;
		INT8	btDriveMode;

		CIA402_OUT()
		{
			usControlWord= 0;
			nTargetPos = 0;
			nTargetVel = 0;
			nOffsetVel = 0;
			sTargetTor = 0;
			sOffsetTor = 0;
			unProfileVel = 0;
			unProfileAcc = 0;
			unProfileDec = 0;
			unQuickStopDec = 0;
			unProfileJerk = 0;
			unMaxProfileVel = 0;
			unMaxProfileAcc = 0;
			btDriveMode = 0;
		}
	};
	struct CIA402_IN //Input 
	{
		UINT16	usStatusWord;
		UINT16	usRawStatusWord;
		INT32	nActualPos;
		INT32	nActualVel;
		INT16	sActualTor;
		UINT32	uActualProfileVel;
		UINT32	uActualProfileAcc;
		UINT32	uActualProfileDec;
		UINT32	uActualProfileJerk;
		UINT32  uActualMaxProfileVel;
		UINT32  uActualMaxProfileAcc;
		UINT32  uActualQuickStopDec;
		INT8	btDriveModeDisplay;	
		UINT16	usErrorCode;
		// INT32   nFollowingError;
		double  dAdditionalPos;
		UINT32  uMotorRatedCurrent;
	
	
		CIA402_IN()
		{
			usStatusWord = 0;
			usRawStatusWord = 0;
			nActualPos = 0;
			nActualVel = 0;
			sActualTor = 0;
			uActualProfileVel = 0;
			uActualProfileAcc = 0;
			uActualProfileDec = 0;
			uActualProfileJerk = 0;
			uActualMaxProfileVel = 0;
			uActualMaxProfileAcc = 0;
			uActualQuickStopDec = 0;
			btDriveModeDisplay = 0;
			usErrorCode = 0;
			dAdditionalPos = 0;
			uMotorRatedCurrent = 0;
			// nFollowingError = 0;
		}
	};
	struct CIA402_SLAVE_PARAMS
	{
		CIA402_OUT	stOutPDOs;
		CIA402_IN	stInPDOs;		
		
		SET_OFFSET(ControlWord); //e.g., UINT32 offControlWord;
		SET_OFFSET(TargetPos);
		SET_OFFSET(TargetVel);
		SET_OFFSET(OffsetVel);
		SET_OFFSET(TargetTor);
		SET_OFFSET(OffsetTor);
		SET_OFFSET(ProfileVel);
		SET_OFFSET(ProfileAcc);
		SET_OFFSET(ProfileDec);
		SET_OFFSET(QuickStopDec);
		SET_OFFSET(ProfileJerk);
		SET_OFFSET(MaxProfileVel);
		SET_OFFSET(MaxProfileAcc);
		SET_OFFSET(DriveMode);
		SET_OFFSET(StatusWord); 
		SET_OFFSET(ActualPos);
		SET_OFFSET(ActualVel);
		SET_OFFSET(ActualTor);
		SET_OFFSET(ErrorCode);
		SET_OFFSET(DriveModeDisplay);
		// SET_OFFSET(FollowingError);
		SET_OFFSET(AdditionalPos);
		SET_OFFSET(MotorRatedCurrent);
		
		SET_BITPOS(ControlWord); //e.g., UINT32 bitControlWord;
		SET_BITPOS(TargetPos);
		SET_BITPOS(TargetVel);
		SET_BITPOS(OffsetVel);
		SET_BITPOS(TargetTor);
		SET_BITPOS(OffsetTor);
		SET_BITPOS(ProfileVel);
		SET_BITPOS(ProfileAcc);
		SET_BITPOS(ProfileDec);
		SET_BITPOS(QuickStopDec);
		SET_BITPOS(ProfileJerk);
		SET_BITPOS(MaxProfileVel);
		SET_BITPOS(MaxProfileAcc);
		SET_BITPOS(DriveMode);
		SET_BITPOS(StatusWord); 
		SET_BITPOS(ActualPos);
		SET_BITPOS(ActualVel);
		SET_BITPOS(ActualTor);
		SET_BITPOS(ErrorCode);
		SET_BITPOS(DriveModeDisplay);
		SET_BITPOS(AdditionalPos);
		SET_BITPOS(MotorRatedCurrent);
	};
	unsigned short m_usStatusWordPrev;

protected:
	virtual BOOL	RegisterPDO			(	);
	virtual UINT16	AnalyzeStatusWord	(UINT16, BOOL abExtend = TRUE);
	virtual void	SetServoStatus		(eSERVO_STATUS aeStatus);
	eSERVO_STATUS	GetServoStatus		(	)					{ return m_eServoStatus; }
	virtual	void	InitSyncs();
	

public:
	CSlaveCIA402Base();
	virtual ~CSlaveCIA402Base();


	virtual void DeInit				(	);
	virtual void SetControlWord		(UINT16	ausControlWord){m_stSlaveParams.stOutPDOs.usControlWord=ausControlWord;};
	virtual void SetTargetVelPDO	(INT32	anTargetVel){m_stSlaveParams.stOutPDOs.nTargetVel=anTargetVel;};
	virtual void SetTargetPosPDO	(INT32	anTargetPos){m_stSlaveParams.stOutPDOs.nTargetPos=anTargetPos;};
	virtual BOOL SetTargetPos		(INT32, BOOL abForced = TRUE, BOOL abRelative = FALSE);
	virtual BOOL SetTargetVel		(INT32, BOOL abForced = FALSE);
	virtual void SetTargetTor		(INT16	asTargetTor){m_stSlaveParams.stOutPDOs.sTargetTor=asTargetTor;};
	virtual void SetProfileVel		(UINT32	aunProfileVel){m_stSlaveParams.stOutPDOs.unProfileVel=aunProfileVel;};
	virtual void SetProfileAcc		(UINT32	aunProfileAcc){m_stSlaveParams.stOutPDOs.unProfileAcc=aunProfileAcc;};
	virtual void SetProfileDec		(UINT32	aunProfileDec) { m_stSlaveParams.stOutPDOs.unProfileDec = aunProfileDec; };
	virtual void SetProfileJerk		(UINT32	aunProfileJerk){m_stSlaveParams.stOutPDOs.unProfileJerk=aunProfileJerk;};
	virtual void SetMaxProfileVel	(UINT32 aunMaxProfileVel){ m_stSlaveParams.stOutPDOs.unMaxProfileVel = aunMaxProfileVel; }
	virtual void SetMaxProfileAcc	(UINT32 aunMaxProfileAcc){ m_stSlaveParams.stOutPDOs.unMaxProfileAcc = aunMaxProfileAcc; }
	virtual void SetQuickStopDec	(UINT32 aunQuickStopDec) { m_stSlaveParams.stOutPDOs.unQuickStopDec = aunQuickStopDec; }
	virtual void SetDriveMode		(INT8	abtDriveMode){ m_stSlaveParams.stOutPDOs.btDriveMode= ValidateCIA402DriveMode(abtDriveMode); }

	/* Servo Status */
	inline BOOL	IsServoOn() { return m_bIsServoOn; };
	inline virtual void	SetServoOnOff		(BOOL abIsServoOnOff) { m_bIsSetServoOnOff = abIsServoOnOff; };
	virtual BOOL SetServoOff			(	);
	virtual BOOL SetServoOn			(INT8	abtDriveMode = CIA402_PROFILE_POSITION);
	
	inline UINT16	GetControlWord	(	){return m_stSlaveParams.stOutPDOs.usControlWord;};
	inline INT32	GetTargetVel	(	){return m_stSlaveParams.stOutPDOs.nTargetVel;};
	inline INT32	GetTargetPos	(	){return m_stSlaveParams.stOutPDOs.nTargetPos;};
	inline INT16	GetTargetTor	(	){return m_stSlaveParams.stOutPDOs.sTargetTor;};
	inline UINT32	GetProfileVel	(	){return m_stSlaveParams.stOutPDOs.unProfileVel;};
	inline UINT32	GetProfileAcc	(	){return m_stSlaveParams.stOutPDOs.unProfileAcc;};
	inline UINT32	GetProfileDec	(	){ return m_stSlaveParams.stOutPDOs.unProfileDec; };
	inline UINT32	GetProfileJerk	(	){return m_stSlaveParams.stOutPDOs.unProfileJerk;};
	inline INT8		GetDriveMode	(	){return m_stSlaveParams.stOutPDOs.btDriveMode;};
	
	inline INT32	GetStartPos		(	){ return m_nStartPos; }
	inline INT32	GetStartVel		(	){ return m_nStartVel; }
	inline INT16	GetStartTor		(	){ return m_uStartTor; }

	inline UINT16	GetStatusWord			(	){return m_stSlaveParams.stInPDOs.usStatusWord;};
	inline UINT16	GetRawStatusWord		(	){return m_stSlaveParams.stInPDOs.usRawStatusWord;};
	inline INT32	GetActualVel			(	){return m_stSlaveParams.stInPDOs.nActualVel;};
	inline INT32	GetActualPos			(	){return m_stSlaveParams.stInPDOs.nActualPos;};
	inline INT16	GetActualTor			(	){return m_stSlaveParams.stInPDOs.sActualTor;};
	inline UINT32	GetActualProfileVel		(	){ return m_stSlaveParams.stInPDOs.uActualProfileVel; };
	inline UINT32	GetActualProfileAcc		(	){ return m_stSlaveParams.stInPDOs.uActualProfileAcc; };
	inline UINT32	GetActualProfileDec		(	){ return m_stSlaveParams.stInPDOs.uActualProfileDec; };
	inline UINT32	GetActualProfileJerk	(	){ return m_stSlaveParams.stInPDOs.uActualProfileJerk; };
	inline UINT32	GetActualMaxProfileVel	(	){ return m_stSlaveParams.stInPDOs.uActualMaxProfileVel; };
	inline UINT32	GetActualMaxProfileAcc	(	){ return m_stSlaveParams.stInPDOs.uActualMaxProfileAcc; };
	inline UINT32	GetActualQuickStopDec	(	){ return m_stSlaveParams.stInPDOs.uActualQuickStopDec; };
	inline INT8		GetActualDriveMode		(	){return m_stSlaveParams.stInPDOs.btDriveModeDisplay;};
	inline UINT16	GetErrorCode			(	){return m_stSlaveParams.stInPDOs.usErrorCode;}

	inline double	GetAdditionalPos		(	){return m_stSlaveParams.stInPDOs.dAdditionalPos;};

	virtual BOOL	IsRelativePos		(	);
	virtual BOOL	IsEndlessMovement	(	){ return m_bIsEndlessMovement; }
	virtual BOOL	SetRelativePosMode	(BOOL abRelative = TRUE);
	virtual BOOL	SetEndlessMovement	(BOOL abEndless = TRUE);


	/* CIA402 Control Word */
	virtual BOOL	SetEmgStop			(	);
	virtual BOOL	SetNewSetPoint		(BOOL abForced = FALSE);
	virtual BOOL	SetHalt				(BOOL abHalt = TRUE);
	virtual void	GoHome				(	);
	
	/* CIA402 Status Word */
	inline BOOL IsReachedTarget		(	){return m_bIsReachedTarget;};
	inline BOOL IsReachedHome		(	){return m_bIsReachedHome;};
	inline BOOL	IsSetPointAck		(	){return m_bIsSetPointAck;};
	inline BOOL IsFollowingError	(	){return m_bIsFollowingError;};
	inline BOOL IsHomingAborted 	(	){return m_bIsHomingAborted;};
	inline BOOL IsHomeSet 			(	){return m_bIsHomeSet;};
	inline BOOL IsHalt 				(	){return m_bIsHalt;};

	/* EtherCAT */
	virtual void WriteToSlave();
	virtual void ReadFromSlave();

	/* Callback Functions */
	void RegisterCallbackStateChange	(CALLBACK_FN afnCallback);
	void RegisterCallbackParamUpdate	(CALLBACK_FN afnCallback);


protected:
	BOOL	m_bIsServoOn;
	BOOL	m_bIsHomingAborted;
	BOOL	m_bIsSetServoOnOff;
	BOOL	m_bIsReachedHome;
	BOOL	m_bIsEndlessMovement;
	ePOSITION_MODE m_ePosMode;
	
	BOOL	m_bIsReachedTarget;
	BOOL	m_bIsSpeedZero;
	BOOL	m_bIsSpeedSaturated;
	BOOL	m_bIsHomeSet;
	BOOL	m_bIsHalt;
	
	BOOL	m_bIsSetPointAck;
	BOOL	m_bIsFollowingError;
	BOOL	m_bIsSetGoHome;
	BOOL	m_bTriggerGoHome;
	CIA402_SLAVE_PARAMS m_stSlaveParams;
	eSERVO_STATUS m_eServoStatus;
	ST_RAW_DATA m_stCurrRawData;


	CALLBACK_FN	m_pStateChange;
	CALLBACK_FN m_pUpdateParam;

private:
	BOOL	m_bFault;

protected:
	INT32	m_nStartPos;
	INT32	m_nStartVel;
	INT16	m_uStartTor;


protected:
	/* todo: make this configurable */
	ec_pdo_entry_info_t cia402_pdo_entries[11] = {
		{0x6040, 0x00, 16},	// control word
		{0x607A, 0x00, 32},	// target position
		{0x60FF, 0x00, 32}, // target velocity
		{0x6071, 0x00, 16}, // target torque
		// {0x6081, 0x00, 32}, // profile velocity
		// {0x6083, 0x00, 32}, // profile acceleration
		// {0x6084, 0x00, 32}, // profile deceleration
		// {0x6085, 0x00, 32}, // quick-stop deceleration
		// {0x607F, 0x00, 32}, // max-profile velocity
		// {0x60C5, 0x00, 32}, // max-profile acceleration/deceleration
		{0x6060, 0x00,  8}, // mode of operation
		{0x6041, 0x00, 16},	// status word
		{0x6064, 0x00, 32},	// actual position (inc)
		{0x606C, 0x00, 32}, // actual velocity
		{0x6077, 0x00, 16}, // actual torque
		{0x6061, 0x00,  8}, // mode of operation display
		// {0x2fe4, 0x02,  64}, // additional pos (abs)
		// {0x6075, 0x00,  32}, // motor rated current
		// {0x60f4, 0x00,  32},
		// {0x603F, 0x00, 16}, // error code
	};
	ec_pdo_info_t cia402_pdos[2] = {
		{0x1600,	5,	cia402_pdo_entries + 0},
		{0x1a00,	5,	cia402_pdo_entries + 5},		

	};
	ec_sync_info_t cia402_syncs[5];

}; //CEcatCIA402Base

typedef CSlaveCIA402Base CSlaveCIA402;

#endif //_ECAT_SLAVE_CIA_402_BASE_
