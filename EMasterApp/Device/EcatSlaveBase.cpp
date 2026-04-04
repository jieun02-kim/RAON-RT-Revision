/*****************************************************************************
*	Name: EcatSlaveBase.cpp
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Implementation of the CEcatSlaveBase class.
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
#include "EcatSlaveBase.h"
//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
CEcatSlaveBase::CEcatSlaveBase()
{
    m_bIsInitialized = FALSE;
    m_bIsDCEnabled = FALSE;
}

CEcatSlaveBase::~CEcatSlaveBase()
{
    
}
//////////////////////////////////////////////////////////////////////
// Slave State
//////////////////////////////////////////////////////////////////////
ECAT_SLAVE_CONFIG_STATE
CEcatSlaveBase::GetSlaveConfigStateSt(  )
{
    return m_stSlaveState;
}

void
CEcatSlaveBase::UpdateSlaveConfigState()
{
    ECAT_SLAVE_CONFIG_STATE stslavestate;
    ecrt_slave_config_state(m_pEcatSlaveConfig, &stslavestate);
    m_stSlaveState = stslavestate;
}

ECAT_STATE_MACH
CEcatSlaveBase::GetSlaveState(  )
{
    return (ECAT_STATE_MACH)(m_stSlaveState.al_state);
}

BOOL
CEcatSlaveBase::IsSlaveOp(  )
{
    BOOL bRet = TRUE;
    ECAT_STATE_MACH stslavestatemach = GetSlaveState();
    if (eOP != stslavestatemach)
        bRet = FALSE;
    
    return bRet;
}

BOOL
CEcatSlaveBase::IsSlaveOnline(  )
{
    ECAT_SLAVE_CONFIG_STATE stslavestate = GetSlaveConfigStateSt();
    return stslavestate.online;
}

//////////////////////////////////////////////////////////////////////
// Slave Configuration
//////////////////////////////////////////////////////////////////////
void
CEcatSlaveBase::SetDeviceType(ECAT_DEVICE_TYPE aeDeviceType /*=eUNKNOWN*/)
{
    m_eDeviceType = aeDeviceType;
    ECAT_SLAVE_TYPE eslavetype;
    switch( m_eDeviceType )
    {
        case    eDIGITAL_IN:
        case    eANALOG_IN:     eslavetype = eIN_ONLY;
                                break;
        case    eANALOG_OUT:
        case    eDIGITAL_OUT:   eslavetype = eOUT_ONLY;
                                break;
        case    eANALOG_IN_OUT:
        case    eDIGITAL_IN_OUT:
        case    eFTSensor:
        case    eCIA402:        eslavetype = eIN_OUT;
                                break;                                
        case    eTERMINAL:
        case    eUNKNOWN:
        default:                eslavetype = eNO_IO;
                                break;
    }
    SetSlaveType( eslavetype );
}

UINT32
CEcatSlaveBase::RegisterPDOEntry(UINT16 asIndex, UINT8 abtSubIndex, UINT32* anBitPos,ECAT_COMM_DIR anEcatCommDir /*=eInput*/)
{
    UINT32 unRet = -1;
    /* ecrt_slave_config_reg_pdo_entry()
    * \retval >=0 Success: Offset of the PDO entry's process data.
    * \retval  <0 Error code.
    */
    if ( TRUE == m_bIsInitialized )
    {
        PECAT_DOMAIN pEcatDomain = m_stEcatMasterDesc.pEcatDomainIn;
        if (eInput != anEcatCommDir)
            pEcatDomain = m_stEcatMasterDesc.pEcatDomainOut;

        unRet = ecrt_slave_config_reg_pdo_entry(m_pEcatSlaveConfig, asIndex, abtSubIndex, pEcatDomain, anBitPos);
        if(unRet < 0)
            DBG_LOG_ERROR("[%d:%d] Register PDO(index:%x subindex:%x) Failed!",m_stEcatSlaveInfo.usAlias, m_stEcatSlaveInfo.usPosition, asIndex, abtSubIndex);
    }
    else    
    {
        DBG_LOG_ERROR("Register PDO Entry before Init( )! Run Init( ) first!");
    }
    return unRet;
}

BOOL
CEcatSlaveBase::EnableDC(UINT32 aunCyleTime)
{
    BOOL bRet = TRUE;
    if( TRUE == m_bIsInitialized )
    {
        if ( TRUE == IsDCSupported())
        {
            ecrt_slave_config_dc(m_pEcatSlaveConfig, GetDCActivateWord(), aunCyleTime, GetDCShiftTime(), 0, 0);
            m_bIsDCEnabled = TRUE;
        }
        else
        {
            /* The AssignActivate word is vendor-specific and can be taken from the XML
            * device description file (Device -> Dc -> AssignActivate). Set this to zero,
            * if the slave shall be operated without distributed clocks (default).
            */
            ecrt_slave_config_dc(m_pEcatSlaveConfig, 0, 0, 0, 0, 0);
        }
    }
    else
    {
        DBG_LOG_ERROR("EnableDC before Init( )! Run Init( ) first!");
        bRet = FALSE;
    }
    SetDCCycleTime(aunCyleTime);
    return bRet;
}

BOOL
CEcatSlaveBase::SetAsDCRef(    )
{
    BOOL bRet = FALSE;
    if ( TRUE == m_bIsInitialized )
    {
        if ( TRUE == IsDCEnabled( ) && 0 == ecrt_master_select_reference_clock(m_stEcatMasterDesc.pEcatMaster, m_pEcatSlaveConfig) )
            bRet = TRUE;
        else
            DBG_LOG_ERROR("SetAsDCRef before DCEnable( )! Run EnableDC( ) first!");
    }
    else
        DBG_LOG_ERROR("SetAsDCRef before Init( )! Run Init( ) first!");
    
    return bRet;
}
//////////////////////////////////////////////////////////////////////
// Slave Operation
//////////////////////////////////////////////////////////////////////
BOOL
CEcatSlaveBase::Init(stEcatMasterDesc astEcatMasterDesc)
{
    m_stEcatMasterDesc = astEcatMasterDesc;
    /* Master-Slave Configuration */
    DBG_LOG_INFO("[%d:%d] Master-Slave Configuration (0x%08x:0x%08x)!",m_stEcatSlaveInfo.usAlias, m_stEcatSlaveInfo.usPosition, m_stEcatSlaveInfo.unVendorID, m_stEcatSlaveInfo.unProdCode);
    m_pEcatSlaveConfig = ecrt_master_slave_config(m_stEcatMasterDesc.pEcatMaster, m_stEcatSlaveInfo.usAlias, m_stEcatSlaveInfo.usPosition, m_stEcatSlaveInfo.unVendorID, m_stEcatSlaveInfo.unProdCode);
    if (NULL == m_pEcatSlaveConfig)
    {
        DBG_LOG_ERROR("[%d:%d] Master-Slave Configuration Failed!",m_stEcatSlaveInfo.usAlias, m_stEcatSlaveInfo.usPosition);
        return FALSE;
    }

    /*DBG_LOG_WARN("[%d:%d] ecrt_master_slave_config-%d!", m_stEcatSlaveInfo.usAlias, m_stEcatSlaveInfo.usPosition, m_pEcatSlaveConfig);*/

    m_bIsInitialized = TRUE;

    if (eNO_IO != GetSlaveType())
    {
        /* PDO Configuration */
        DBG_LOG_INFO("[%d:%d] PDO Configuration!",m_stEcatSlaveInfo.usAlias, m_stEcatSlaveInfo.usPosition);

        /* Init Syncs */
        InitSyncs();
        
        if (NULL == m_stEcatSlaveInfo.pEcatSyncInfo)
        {
            DBG_LOG_ERROR("[%d:%d] Sync Manager Not Configured!", m_stEcatSlaveInfo.usAlias, m_stEcatSlaveInfo.usPosition);
            return FALSE;
        }
        
        int nRet = ecrt_slave_config_pdos(m_pEcatSlaveConfig, EC_END, m_stEcatSlaveInfo.pEcatSyncInfo);
        if (0 != nRet)
        {
            DBG_LOG_ERROR("[%d:%d] PDO Configuration Failed!",m_stEcatSlaveInfo.usAlias, m_stEcatSlaveInfo.usPosition);
            return FALSE;
        }

        DBG_LOG_INFO("[%d:%d] Registering PDO!",m_stEcatSlaveInfo.usAlias, m_stEcatSlaveInfo.usPosition);
        BOOL bRet = RegisterPDO();
        if (bRet == FALSE)
        {
            DBG_LOG_ERROR("[%d:%d] Registering PDO Failed!",m_stEcatSlaveInfo.usAlias, m_stEcatSlaveInfo.usPosition);
            return FALSE;
        }

    }
    return TRUE;
}

void
CEcatSlaveBase::SetVendorInfo(UINT32 auVendorID, UINT32 auProductCode)
{
    SetVendorID(auVendorID);
    SetProductCode(auProductCode);
}

void
CEcatSlaveBase::SetDCInfo(BOOL abDCSupported, UINT16 ausActivateWord, INT32 anShiftTime)
{
    SetDCSupported(abDCSupported);
    SetDCActivateWord(ausActivateWord);
    SetDCShiftTime(anShiftTime);
}

BOOL
CEcatSlaveBase::InitSlave(UINT16 ausAlias, UINT16 ausPosition, stEcatMasterDesc astEcatMasterDesc)
{
    BOOL bRet = TRUE;
    SetAlias(ausAlias);
    SetPosition(ausPosition);
    
    bRet = Init(astEcatMasterDesc);
    if (bRet == FALSE)
        DBG_LOG_ERROR("[%d:%d] InitSlave Failed!",m_stEcatSlaveInfo.usAlias, m_stEcatSlaveInfo.usPosition);

    return bRet;
}

void
CEcatSlaveBase::ActivateDomainPD(stEcatMasterDesc astEcatMasterDesc)
{
    m_stEcatMasterDesc.pEcatDomainPdIn = astEcatMasterDesc.pEcatDomainPdIn;
    m_stEcatMasterDesc.pEcatDomainPdOut = astEcatMasterDesc.pEcatDomainPdOut;
}

BOOL
CEcatSlaveBase::WritePdoS(UINT32 aunOffset, INT64 anValue, int anBitSize /*=64*/)
{
    switch (anBitSize)
    {
        case 8  :
            EC_WRITE_S8(m_stEcatMasterDesc.pEcatDomainPdOut+aunOffset, (INT8)anValue);
            break;
        case 16 :
            EC_WRITE_S16(m_stEcatMasterDesc.pEcatDomainPdOut+aunOffset, (INT16)anValue);
            break;
        case 32 :
            EC_WRITE_S32(m_stEcatMasterDesc.pEcatDomainPdOut+aunOffset, (INT32)anValue);
            break;
        case 64 :
            EC_WRITE_S64(m_stEcatMasterDesc.pEcatDomainPdOut+aunOffset, (INT64)anValue);
            break;
        default:
            return FALSE;
    }
    return TRUE;
}

BOOL
CEcatSlaveBase::WritePdoU(UINT32 aunOffset, UINT64 anValue, int anBitSize /*=64*/)
{
    switch (anBitSize)
    {
        case 8  :
            EC_WRITE_U8(m_stEcatMasterDesc.pEcatDomainPdOut+aunOffset, (UINT8)anValue);
            break;
        case 16 :
            EC_WRITE_U16(m_stEcatMasterDesc.pEcatDomainPdOut+aunOffset, (UINT16)anValue);
            break;
        case 32 :
            EC_WRITE_U32(m_stEcatMasterDesc.pEcatDomainPdOut+aunOffset, (UINT32)anValue);
            break;
        case 64 :
            EC_WRITE_U64(m_stEcatMasterDesc.pEcatDomainPdOut+aunOffset, (UINT64)anValue);
            break;
        default:
            return FALSE;
    }
    return TRUE;
}

UINT8                   
CEcatSlaveBase::ReadPdoU8(UINT32 aunOffset, BOOL abIsOut)
{ 
    if (abIsOut)
        return EC_READ_U8(m_stEcatMasterDesc.pEcatDomainPdOut + aunOffset);

    return EC_READ_U8(m_stEcatMasterDesc.pEcatDomainPdIn + aunOffset);
}

UINT16                  
CEcatSlaveBase::ReadPdoU16(UINT32 aunOffset, BOOL abIsOut)
{ 
    if (abIsOut)
        return EC_READ_U16(m_stEcatMasterDesc.pEcatDomainPdOut + aunOffset);

    return EC_READ_U16(m_stEcatMasterDesc.pEcatDomainPdIn + aunOffset); 
}

UINT32                  
CEcatSlaveBase::ReadPdoU32(UINT32 aunOffset, BOOL abIsOut)
{ 
    if (abIsOut)
        return EC_READ_U32(m_stEcatMasterDesc.pEcatDomainPdOut + aunOffset);
    
    return EC_READ_U32(m_stEcatMasterDesc.pEcatDomainPdIn + aunOffset); 
}

UINT64                  
CEcatSlaveBase::ReadPdoU64(UINT32 aunOffset, BOOL abIsOut)
{ 
    if (abIsOut)
        return EC_READ_U64(m_stEcatMasterDesc.pEcatDomainPdOut + aunOffset);

    return EC_READ_U64(m_stEcatMasterDesc.pEcatDomainPdIn + aunOffset); 
}

INT8                    
CEcatSlaveBase::ReadPdoS8(UINT32 aunOffset, BOOL abIsOut)
{ 
    if (abIsOut)
        return EC_READ_S8(m_stEcatMasterDesc.pEcatDomainPdOut + aunOffset);

    return EC_READ_S8(m_stEcatMasterDesc.pEcatDomainPdIn + aunOffset); 
}

INT16                   
CEcatSlaveBase::ReadPdoS16(UINT32 aunOffset, BOOL abIsOut)
{ 
    if (abIsOut)
        return EC_READ_S16(m_stEcatMasterDesc.pEcatDomainPdOut + aunOffset);

    return EC_READ_S16(m_stEcatMasterDesc.pEcatDomainPdIn + aunOffset); 
}

INT32                   
CEcatSlaveBase::ReadPdoS32(UINT32 aunOffset, BOOL abIsOut)
{ 
    if (abIsOut)
        return EC_READ_S32(m_stEcatMasterDesc.pEcatDomainPdOut + aunOffset);

    return EC_READ_S32(m_stEcatMasterDesc.pEcatDomainPdIn + aunOffset); 
}

INT64                   
CEcatSlaveBase::ReadPdoS64(UINT32 aunOffset, BOOL abIsOut)
{ 
    if (abIsOut)
        return EC_READ_S64(m_stEcatMasterDesc.pEcatDomainPdOut + aunOffset);

    return EC_READ_S64(m_stEcatMasterDesc.pEcatDomainPdIn + aunOffset); 
}

double CEcatSlaveBase::ReadPdoDouble(UINT32 aunOffset, BOOL abIsOut)
{
    if (abIsOut)
        return EC_READ_LREAL(m_stEcatMasterDesc.pEcatDomainPdOut + aunOffset);

    return EC_READ_LREAL(m_stEcatMasterDesc.pEcatDomainPdIn + aunOffset);
}