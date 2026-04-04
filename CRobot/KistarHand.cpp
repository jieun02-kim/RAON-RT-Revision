#include "KistarHand.h"


CKistarHand::CKistarHand(INT32 nNoAxes, BOOL abEnabled, BOOL abConnected)     
{
    m_bConnected = abConnected;

	if (FALSE == m_bConnected) 
        SetEnabled(FALSE);

    // m_pEcMaster = NULL;
    m_pstAxisLimits = NULL;
    m_pstAxisLimits = new ST_AXIS_LIMITS[nNoAxes];
    
    
}


CKistarHand::~CKistarHand()
{
    delete[] m_pstAxisLimits;
    m_pstAxisLimits = NULL;
}

BOOL CKistarHand::Init(CEcatMaster& apEcmaster)
{
    m_pEcMaster = &apEcmaster;
    m_pEcMaster->AddSlave(&m_cEcSlave);
    return TRUE;
}

void CKistarHand::SetTargetPos(int nAxis, double dTarget)
{
    m_cEcSlave.SetTargetPos(nAxis, RAD2USR(dTarget));
}

double CKistarHand::GetTargetPos(int nAxis)
{
    double dTarget = m_cEcSlave.GetTargetPosition(nAxis);

    return USR2RAD(dTarget);
}

void CKistarHand::SetPositionLimits(INT32 nAxis, double dPosLimitL, double dPosLimitU, BOOL abSet)
{
    m_pstAxisLimits[nAxis].stPos.dLower = ConvertDeg2Rad(dPosLimitL);
    m_pstAxisLimits[nAxis].stPos.dUpper = ConvertDeg2Rad(dPosLimitU);
    m_pstAxisLimits[nAxis].bIsSet = abSet;
}


