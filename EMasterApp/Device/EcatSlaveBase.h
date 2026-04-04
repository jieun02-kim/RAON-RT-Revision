/*****************************************************************************
*	Name: EcatSlaveBase.h
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Header for the CEcatSlaveBase class.
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
#ifndef _ECAT_SLAVE_BASE_
#define _ECAT_SLAVE_BASE_

#include "EcatCommon.h"

#define SET_PREFIX_VAR(T, P, N) __typeof__(T) P## N
#define GET_PREFIX_VAR(P, N)	P## N
#define SET_OFFSET(N) SET_PREFIX_VAR(UINT32,off,N)
#define GET_OFFSET(N) GET_PREFIX_VAR(off,N)
#define SET_BITPOS(N) SET_PREFIX_VAR(UINT32,bit,N)	
#define GET_BITPOS(N) GET_PREFIX_VAR(bit,N)

class CEcatSlaveBase
{

public:
    CEcatSlaveBase();
	virtual~CEcatSlaveBase();

private:
    BOOL                    m_bIsInitialized; 
    BOOL                    m_bIsDCEnabled;
    void                    SetSlaveType            (ECAT_SLAVE_TYPE aeSlaveType){m_stEcatSlaveInfo.eSlaveType=aeSlaveType;}
    ECAT_SLAVE_TYPE         GetSlaveType            (   ){return m_stEcatSlaveInfo.eSlaveType;}

protected:
    UINT32                  RegisterPDOEntry        (UINT16, UINT8, UINT32*, ECAT_COMM_DIR anEcatCommDir = eInput);
    BOOL                    Init                    (stEcatMasterDesc);
    void                    SetVendorID             (UINT32 aunVendorId) { m_stEcatSlaveInfo.unVendorID = aunVendorId; }
    void                    SetProductCode          (UINT32 aunProdCode) { m_stEcatSlaveInfo.unProdCode = aunProdCode; }
    void                    SetDCSupported          (BOOL abIsDCSupported){m_stEcatSlaveInfo.stEcatDCInfo.bIsSupported=abIsDCSupported;}
    void                    SetDCActivateWord       (UINT16 ausActivateWord){m_stEcatSlaveInfo.stEcatDCInfo.usActivateWord=ausActivateWord;}
    void                    SetDCCycleTime          (UINT32 aunDCCycleTime){m_stEcatSlaveInfo.stEcatDCInfo.unCycleTime=aunDCCycleTime;}
    void                    SetDCShiftTime          (INT32 anDCShiftTime){m_stEcatSlaveInfo.stEcatDCInfo.nShiftTime=anDCShiftTime;}
    void                    SetEcatPdoSync          (PECAT_SYNC_INFO apEcatSyncInfo){m_stEcatSlaveInfo.pEcatSyncInfo=apEcatSyncInfo;}
    void                    SetDeviceType           (ECAT_DEVICE_TYPE aeDeviceType = eUNKNOWN);
    virtual void            InitSyncs               (   ) { return; } //todo: make this smart

    ECAT_SLAVE_CONFIG_STATE GetSlaveConfigStateSt   (   );

    /* Register PDO syncs from slaves (use ethercat cstruct)
     * should be re-written depending on the slave 
    */
    virtual BOOL            RegisterPDO             (   ){return TRUE;}

public:
    void                    SetVendorInfo           (UINT32 auVendorID, UINT32 auProductCode);
    void                    SetDCInfo               (BOOL abDCSupported, UINT16 ausActivateWord, INT32 anShiftTime);
    BOOL                    InitSlave               (UINT16 ausAlias, UINT16 ausPosition, stEcatMasterDesc astEcatMasterDesc);

    virtual void            DeInit() {};
    void                    ActivateDomainPD        (stEcatMasterDesc);
    BOOL                    EnableDC                (UINT32);
    BOOL                    SetAsDCRef              (   );
    ECAT_STATE_MACH         GetSlaveState           (   );
    UINT32                  GetVendorID             (   ){return m_stEcatSlaveInfo.unVendorID;}
    UINT32                  GetProductCode          (   ){return m_stEcatSlaveInfo.unProdCode;}
    void                    SetAlias                (UINT16 ausAlias) { m_stEcatSlaveInfo.usAlias = ausAlias; }
    void                    SetPosition             (UINT16 ausPosition) { m_stEcatSlaveInfo.usPosition = ausPosition; }
    UINT16                  GetAlias                (   ){return m_stEcatSlaveInfo.usAlias;}
    UINT16                  GetPosition             (   ){return m_stEcatSlaveInfo.usPosition;}
    UINT16                  GetDCActivateWord       (   ){return m_stEcatSlaveInfo.stEcatDCInfo.usActivateWord;}
    INT32                   GetDCShiftTime          (   ){return m_stEcatSlaveInfo.stEcatDCInfo.nShiftTime;}
    UINT32                  GetDCCycleTime          (   ){return m_stEcatSlaveInfo.stEcatDCInfo.unCycleTime;}
    PECAT_SYNC_INFO         GetEcatPdoSync          (   ){return m_stEcatSlaveInfo.pEcatSyncInfo;}
    ECAT_DEVICE_TYPE        GetDeviceType           (   ){return m_eDeviceType;}

    BOOL                    IsSlaveOp               (   );
    BOOL                    IsSlaveOnline           (   );
    BOOL                    IsDCEnabled             (   ){return m_bIsDCEnabled;};
    BOOL                    IsDCSupported           (   ){return m_stEcatSlaveInfo.stEcatDCInfo.bIsSupported;};
    BOOL                    IsInitialized           (   ){return m_bIsInitialized;};

    /* TODO: make an enum to combine these two functions */
    BOOL                    WritePdoU               (UINT32, UINT64, int anBitSize=64);  // unsigned 
    BOOL                    WritePdoS               (UINT32, INT64, int anBitSize=64);  // signed

    /* TODO: EC_READ_*() Function wrappers */
    UINT8                   ReadPdoU8               (UINT32, BOOL bIsOut = FALSE); 
    UINT16                  ReadPdoU16              (UINT32, BOOL bIsOut = FALSE);
    UINT32                  ReadPdoU32              (UINT32, BOOL bIsOut = FALSE);
    UINT64                  ReadPdoU64              (UINT32, BOOL bIsOut = FALSE);
    INT8                    ReadPdoS8               (UINT32, BOOL bIsOut = FALSE);
    INT16                   ReadPdoS16              (UINT32, BOOL bIsOut = FALSE);
    INT32                   ReadPdoS32              (UINT32, BOOL bIsOut = FALSE);
    INT64                   ReadPdoS64              (UINT32, BOOL bIsOut = FALSE);
    double 					ReadPdoDouble			(UINT32 aunOffset, BOOL abIsOut = FALSE);

    /* Send EtherCAT Data to slaves
     * Should be re-written depending on the slave
    */
    virtual void            WriteToSlave            (   ){};;
    /* Receive EtherCAT Data from slaves
     * Should be re-written depending on the slave
    */
    virtual void            ReadFromSlave           (   ){};;

public:
    void                    UpdateSlaveConfigState();
	/* This should not be used outside of the slave class */
    // PECAT_SLAVE_CONFIG      GetEcatSlaveConfig() { return m_pEcatSlaveConfig; }

protected:
    stEcatSlaveInfo         m_stEcatSlaveInfo;
    ECAT_SLAVE_CONFIG_STATE m_stSlaveState;

private:
    stEcatMasterDesc	    m_stEcatMasterDesc;
    PECAT_SLAVE_CONFIG      m_pEcatSlaveConfig;
    ECAT_DEVICE_TYPE        m_eDeviceType;

}; //CEcatSlaveBase

#endif //_ECAT_SLAVE_BASE_