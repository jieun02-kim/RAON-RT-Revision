/*****************************************************************************
*	Name: EcatMasterBase.cpp
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Implementation of the CEcatMasterBase class.
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
#include "EcatMasterBase.h"


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
CEcatMasterBase::CEcatMasterBase()
{
    m_bInit = FALSE;
    m_bIsDCEnabled = FALSE;
    m_unCycleTime = 0;
}

CEcatMasterBase::~CEcatMasterBase()
{
    
}
//////////////////////////////////////////////////////////////////////
// Master-related
//////////////////////////////////////////////////////////////////////
void
CEcatMasterBase::UpdateMasterStateSt()
{
    ECAT_MASTER_STATE stmasterstate;
    ecrt_master_state(m_stEcatMasterDesc.pEcatMaster, &stmasterstate);
    m_stMasterState = stmasterstate;
}

ECAT_MASTER_STATE
CEcatMasterBase::GetMasterStateSt()
{
    return m_stMasterState;
}

ECAT_STATE_MACH
CEcatMasterBase::GetMasterState()
{
    return (ECAT_STATE_MACH)(m_stMasterState.al_states);     
}

// the overall slave state depends on the state of the slave with the lowest value in the state machine
ECAT_STATE_MACH
CEcatMasterBase::GetSlaveState()
{
    if (GetSlaveCnt() <= 0) return ECAT_STATE_MACH::eNONE;

    
    ECAT_STATE_MACH eMinState = ECAT_STATE_MACH::eOP;
    for (unsigned nCnt = 0; nCnt < GetSlaveCnt(); nCnt++)
    {
        ECAT_STATE_MACH eSlaveState = m_vEcatSlaves[nCnt]->GetSlaveState();
        if (eSlaveState < eMinState) eMinState = eSlaveState;
    }
    return eMinState;
}


BOOL
CEcatMasterBase::IsMasterOp()
{
    BOOL bRet = TRUE;
    ECAT_STATE_MACH ststatemach = GetMasterState();
    if (ststatemach != eOP)
        bRet = FALSE;

    return bRet;
}

BOOL
CEcatMasterBase::IsLinkUp()
{
    bool bRet = FALSE;
    ECAT_MASTER_STATE stmasterstate = GetMasterStateSt();
    if (stmasterstate.link_up)
        bRet = TRUE;

    return bRet;
}

unsigned int
CEcatMasterBase::GetRespSlaveNo()
{
    ECAT_MASTER_STATE stmasterstate = GetMasterStateSt();
    return stmasterstate.slaves_responding;
}

//////////////////////////////////////////////////////////////////////
// Domain-related
//////////////////////////////////////////////////////////////////////
void
CEcatMasterBase::UpdateDomainStateSt(ECAT_COMM_DIR aeInOut)
{
    ECAT_DOMAIN_STATE stdomainstate;
    if (eOutput == aeInOut)
    {
        ecrt_domain_state(m_stEcatMasterDesc.pEcatDomainOut, &stdomainstate);
        m_stDomainOutState = stdomainstate;
    }
    else
    {
        ecrt_domain_state(m_stEcatMasterDesc.pEcatDomainIn, &stdomainstate);
        m_stDomainInState = stdomainstate;
    }
}

ECAT_DOMAIN_STATE
CEcatMasterBase::GetDomainStateSt(ECAT_COMM_DIR aeInOut)
{
    if (aeInOut == eOutput)
        return m_stDomainOutState;
    else
        return m_stDomainInState;
}

ECAT_WC			
CEcatMasterBase::GetWorkingCounter(ECAT_COMM_DIR aeInOut)
{
    ECAT_DOMAIN_STATE stdomainstate = GetDomainStateSt(aeInOut);
    return (ECAT_WC_STATE)(stdomainstate.working_counter);
}

ECAT_WC_STATE
CEcatMasterBase::GetDomainState(ECAT_COMM_DIR aeInOut /*=eInput*/)
{
    ECAT_DOMAIN_STATE stdomainstate = GetDomainStateSt(aeInOut);
    return (ECAT_WC_STATE)(stdomainstate.wc_state); 
}

BOOL
CEcatMasterBase::IsDomainOp(ECAT_COMM_DIR aeInOut /*=eInput*/)
{
    BOOL bRet = TRUE;
    ECAT_WC_STATE stdomainwcstate = GetDomainState(aeInOut);
    if (stdomainwcstate != eWC_COMPLETE)
        bRet = FALSE;

    return bRet;
}

BOOL
CEcatMasterBase::IsAllSlavesOp()
{
    for ( unsigned nCnt = 0; nCnt < GetSlaveCnt(); nCnt++ )
        if ( FALSE == m_vEcatSlaves[nCnt]->IsSlaveOp() )
            return FALSE;

    return TRUE;
}

void
CEcatMasterBase::UpdateAppTime(UINT64 alAppTime)
{
    ecrt_master_application_time(m_stEcatMasterDesc.pEcatMaster, alAppTime);

    if (TRUE == IsDCEnabled())
    {
        ecrt_master_sync_reference_clock(m_stEcatMasterDesc.pEcatMaster);
        ecrt_master_sync_slave_clocks(m_stEcatMasterDesc.pEcatMaster);
    }
}
//////////////////////////////////////////////////////////////////////
// Master Operations
//////////////////////////////////////////////////////////////////////
BOOL
CEcatMasterBase::Init(UINT32 aunCycleTime, BOOL abIsEnableDC /*=FALSE*/ )
{
    if (TRUE == m_bInit)
    {
        DBG_LOG_WARN("(%s) EtherCAT Master has already been initialized!", "CEcatMasterBase");
        return TRUE;
    }
    
    BOOL bRet = FALSE;

    if (0 == GetSlaveCnt())
    {
        DBG_LOG_ERROR("(%s) Slave list is empty!", "CEcatMasterBase");
        return FALSE;
    }
    
    DBG_LOG_INFO("(%s) Initializing EtherCAT master!", "CEcatMasterBase");
    m_stEcatMasterDesc.pEcatMaster = ecrt_request_master(0);
    if (!m_stEcatMasterDesc.pEcatMaster)
    {
        DeInit();
        DBG_LOG_ERROR("(%s) Unable to request Master Instance!", "CEcatMasterBase");
        return FALSE;
    }

    DBG_LOG_INFO("(%s) Creating Input Domain!", "CEcatMasterBase");
    m_stEcatMasterDesc.pEcatDomainIn = ecrt_master_create_domain(m_stEcatMasterDesc.pEcatMaster);
    if (!m_stEcatMasterDesc.pEcatDomainIn)
    {
        DeInit();
        DBG_LOG_ERROR("(%s) Unable to create Input Domain!", "CEcatMasterBase");
        return FALSE;
    }
    DBG_LOG_INFO("(%s) Creating Output Domain!", "CEcatMasterBase");
    m_stEcatMasterDesc.pEcatDomainOut = ecrt_master_create_domain(m_stEcatMasterDesc.pEcatMaster);
    if (!m_stEcatMasterDesc.pEcatDomainOut)
    {
        DeInit();
        DBG_LOG_ERROR("(%s) Unable to create Output Domain!", "CEcatMasterBase");
        return FALSE;
    }

    UpdateMasterStateSt();
    UpdateDomainStateSt(eInput);
    UpdateDomainStateSt(eOutput);

    /* Initiate EtherCAT Slaves */
    m_bIsDCEnabled = abIsEnableDC;
    m_unCycleTime = aunCycleTime;

    DBG_LOG_INFO("(%s) There are %d Responding Slaves!", "CEcatMasterBase", GetRespSlaveNo());
    DBG_LOG_INFO("(%s) Initializing EtherCAT Slaves!", "CEcatMasterBase", GetRespSlaveNo());
    
    bRet = InitSlaves();
    if (FALSE == bRet)
    {
        DeInit();
        DBG_LOG_ERROR("(%s) Unable to Initialize Slaves!", "CEcatMasterBase");
        return FALSE;
    }

    DBG_LOG_INFO("EcatMaster: Activating Master Instance!");
    if ( 0 != ecrt_master_activate(m_stEcatMasterDesc.pEcatMaster))
    {
        DeInit();
        DBG_LOG_ERROR("(%s) Unable to Activate Master!", "CEcatMasterBase");
        return FALSE;
    }
    
    DBG_LOG_INFO("(%s) Activating Input Domain!", "CEcatMasterBase");
    if (!(m_stEcatMasterDesc.pEcatDomainPdIn = ecrt_domain_data(m_stEcatMasterDesc.pEcatDomainIn)))
    {
        DeInit();
        DBG_LOG_ERROR("(%s) Unable to Activate Input Domain!", "CEcatMasterBase");
        return FALSE;
    }

    DBG_LOG_INFO("(%s) Activating Output Domain!", "CEcatMasterBase");
    if (!(m_stEcatMasterDesc.pEcatDomainPdOut = ecrt_domain_data(m_stEcatMasterDesc.pEcatDomainOut)))
    {
        DeInit();
        DBG_LOG_ERROR("(%s) Unable to Activate Output Domain!", "CEcatMasterBase");
        return FALSE;
    }

    /* Activate Domains for all slaves */
    for ( unsigned nCnt = 0; nCnt < GetSlaveCnt(); nCnt++ )
        m_vEcatSlaves[nCnt]->ActivateDomainPD(m_stEcatMasterDesc);

    DBG_LOG_INFO("(%s) EtherCAT Master/Slaves Initialized Successfully!", "CEcatMasterBase");
    
    m_bInit = TRUE;
    return TRUE;
}



void
CEcatMasterBase::AddSlave(CEcatSlaveBase* acEcatSlaveBase)
{
    size_t szSlaveNo = m_vEcatSlaves.size();
    m_vEcatSlaves.push_back(acEcatSlaveBase);
    acEcatSlaveBase->SetAlias((UINT16)0);
    acEcatSlaveBase->SetPosition((UINT16)szSlaveNo);
}

BOOL
CEcatMasterBase::InitSlaves(    ) 
{
    BOOL bRet;
    unsigned nInitSlaves = 0;

    if (0 >= GetRespSlaveNo() || 0 >= GetSlaveCnt())
        return FALSE;

    for ( unsigned nCnt = 0; nCnt < GetSlaveCnt(); nCnt++ )
    {
        bRet = m_vEcatSlaves[nCnt]->InitSlave((UINT16)(0), (UINT16)(nCnt), m_stEcatMasterDesc);

        if ( TRUE == m_bIsDCEnabled )
            m_vEcatSlaves[nCnt]->EnableDC(m_unCycleTime);

        nInitSlaves += bRet;
    }
    if (nInitSlaves != GetRespSlaveNo())
    {
        DBG_LOG_ERROR("(%s) All Connected Slaves should be Initialized (Connected:%d != Initialized: %d)!", "CEcatMasterBase", GetRespSlaveNo(),  nInitSlaves);
        return FALSE;
    }
    
    return TRUE;
}

void
CEcatMasterBase::DeInitSlaves()
{
    if (0 < GetSlaveCnt())
    {
        for (unsigned nCnt = 0; nCnt < GetSlaveCnt(); nCnt++)
        {
            m_vEcatSlaves[nCnt]->DeInit();
        }
        DBG_LOG_INFO("EcatMaster: DeInit Slaves Successful!");
    }
    else
        DBG_LOG_WARN("EcatMaster: No slaves to DeInit!");
}

void
CEcatMasterBase::DeInit()
{
    /* Read and write domains for clean deinit*/
    ReadSlaves();
    DeInitSlaves();
    WriteSlaves(GetCurrTimeInNs());
    if (NULL != m_stEcatMasterDesc.pEcatMaster)
    {
        ecrt_release_master(m_stEcatMasterDesc.pEcatMaster);
        m_stEcatMasterDesc.pEcatMaster = NULL;
        DBG_LOG_INFO("EcatMaster: EtherCAT Master has been released!");
    }
    m_vEcatSlaves.clear();
    m_bInit = FALSE;
}

void
CEcatMasterBase::WriteSlaves(UINT64 alAppTime)
{
    if (m_bInit)
    {
        ecrt_master_receive(m_stEcatMasterDesc.pEcatMaster);
        ecrt_domain_process(m_stEcatMasterDesc.pEcatDomainOut);

        // cyclically, update domain state (output)
        UpdateDomainStateSt(eOutput);

        for (unsigned nCnt = 0; nCnt < GetSlaveCnt(); nCnt++)
        {
			if ((m_vEcatSlaves[nCnt]->GetDeviceType() != eTERMINAL) && (m_vEcatSlaves[nCnt]->GetDeviceType() != eJUNCTION))
			{
				    if (IsDomainOp(eOutput)) m_vEcatSlaves[nCnt]->WriteToSlave();
			}
        }

        UpdateAppTime(alAppTime);
        ecrt_domain_queue(m_stEcatMasterDesc.pEcatDomainOut);
        ecrt_master_send(m_stEcatMasterDesc.pEcatMaster);
    }
    else
        DBG_LOG_WARN("(%s) Trying to write slaves without initializing EtherCAT Master!", "CEcatMasterBase");
}

void
CEcatMasterBase::ReadSlaves(    )
{
    if (m_bInit)
    {
        ecrt_master_receive(m_stEcatMasterDesc.pEcatMaster);
        ecrt_domain_process(m_stEcatMasterDesc.pEcatDomainIn);

        // cyclically, update master and domain state (input)
        UpdateMasterStateSt();
        UpdateDomainStateSt(eInput);

        for (unsigned nCnt = 0; nCnt < GetSlaveCnt(); nCnt++)
        {
            // cyclically, update slave state before reading data
            m_vEcatSlaves[nCnt]->UpdateSlaveConfigState();
			if ((m_vEcatSlaves[nCnt]->GetDeviceType() != eTERMINAL) && (m_vEcatSlaves[nCnt]->GetDeviceType() != eJUNCTION))
			{
                if (IsDomainOp()) m_vEcatSlaves[nCnt]->ReadFromSlave();			
			}
        }
        
        ecrt_domain_queue(m_stEcatMasterDesc.pEcatDomainIn);
        ecrt_master_send(m_stEcatMasterDesc.pEcatMaster);
        
    }
    else
        DBG_LOG_WARN("(%s) Trying to read slaves without initializing EtherCAT Master!", "CEcatMasterBase");
}