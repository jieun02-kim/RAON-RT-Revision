#ifndef __KISTAR_HAND__
#define __KISTAR_HAND__

#include "Axis.h"
#include "SlaveKistarHand.h"
#include "EcatMasterBase.h"

#define RAD2USR(X) ((X)*8192.0/M_PI)
#define USR2RAD(X) ((X)*M_PI/8192.0)

enum eControlMode {
    PWM_MODE = 0x0000,
    POSITION_MODE = 0xFFFF,
};




class CKistarHand
{
public:
    CKistarHand(INT32 nNoAxes, BOOL abEnabled, BOOL abConnected);
    virtual ~CKistarHand();

public:
    BOOL Init(CEcatMaster&);
    BOOL isConnected() { return m_bConnected; }

    void SetEnabled(BOOL bEnabled) { m_bEnabled = bEnabled; }
    void SetServoOn(UINT16 usFlag=0xFFFF) { m_cEcSlave.SetServoStatus(usFlag); }
    void SetServoOff(UINT16 usFlag=0x0000) {m_cEcSlave.SetServoStatus(usFlag); }
    void SetControlMode(UINT16 usMode) { m_cEcSlave.SetControlMode(usMode); }
    void SetTargetPos(int nAxis, double dTarget);
    double GetTargetPos(int nAxis);

    
    void SetName(TSTRING astrName) { m_strName = astrName; }
	TSTRING	GetName() { return m_strName; }
    void SetVendorInfo(UINT32 auVendorID, UINT32 auProductCode) { m_cEcSlave.SetVendorInfo(auVendorID, auProductCode); };
    void SetDCInfo(BOOL abDCSupported, UINT16 ausActivateWord, INT32 anShiftTime) { m_cEcSlave.SetDCInfo(abDCSupported, ausActivateWord, anShiftTime); };
    void SetPositionLimits(INT32 nAxis, double dPosLimitL, double dPosLimitU, BOOL abSet = TRUE);    
    ST_LIMITS GetPositionLimits(INT32 nAxis) { return m_pstAxisLimits[nAxis].stPos; }
    

private:
    BOOL m_bEnabled;
    TSTRING	m_strName;

    ST_AXIS_LIMITS* m_pstAxisLimits;


protected:
	CSlaveKistarHand	m_cEcSlave;
	CEcatMaster*	    m_pEcMaster;
    BOOL                m_bConnected;

    
};


#endif