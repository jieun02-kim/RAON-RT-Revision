/*****************************************************************************
*	Name: SlaveKistFT.h
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Header for the CSlaveKistFT child class.
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/

#ifndef _ECAT_SLAVE_KIST_FT_
#define _ECAT_SLAVE_KIST_FT_

#include "EcatSlaveBase.h"

#ifndef KIST_VENDOR_ID
#define KIST_VENDOR_ID         		0x00828845
#endif
#define KIST_FT_SENSOR 				0x00009252
#define KIST_FT_ACTIVATE_WORD       0x0300
#define KIST_FT_SYNC0_SHIFT         125000

/* Master 0, Slave 2, "IFBOX4NEWIF"
 * Vendor ID:       0x00828845
 * Product code:    0x00009252
 * Revision number: 0x00000001
 */

ec_pdo_entry_info_t slave_2_pdo_entries[] = {
    {0x7000, 0x01, 16}, /* value1 */
    {0x7000, 0x02, 16}, /* value2 */
    {0x6000, 0x01, 32}, /* Fx */
    {0x6000, 0x02, 32}, /* Fy */
    {0x6000, 0x03, 32}, /* Fz */
    {0x6000, 0x04, 32}, /* Tx */
    {0x6000, 0x05, 32}, /* Ty */
    {0x6000, 0x06, 32}, /* Tz */
    {0x6000, 0x07, 32}, /* count */
};

ec_pdo_info_t slave_2_pdos[] = {
    {0x1600, 2, slave_2_pdo_entries + 0}, /* Output mapping 0 */
    {0x1a00, 7, slave_2_pdo_entries + 2}, /* FTValue process data mapping */
};

ec_sync_info_t kist_ft_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_2_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slave_2_pdos + 1, EC_WD_DISABLE},
    {0xff}
};

/* FT Sensor Operation Mode */
typedef enum
{
    DEFAULT_OP = 0,
    ADC_VALUE = 8,
    BIAS_FLASH_SAVE = 9,
    BIAS_NO_SAVE = 6,
} FT_OP_MODE;

class CSlaveKistFT : public CEcatSlaveBase
{
public:
	CSlaveKistFT()
	{
		CEcatSlaveBase::SetVendorID((UINT32)KIST_VENDOR_ID);
		CEcatSlaveBase::SetProductCode((UINT32)KIST_FT_SENSOR);
		CEcatSlaveBase::SetDeviceType(eFTSensor);
        CEcatSlaveBase::SetDCSupported(FALSE);
        //CEcatSlaveBase::SetDCActivateWord(KIST_FT_ACTIVATE_WORD);
        //CEcatSlaveBase::SetDCShiftTime((INT32)KIST_FT_SYNC0_SHIFT);
        CEcatSlaveBase::SetEcatPdoSync(kist_ft_syncs);
	};
	~CSlaveKistFT() {};


public:

    inline void SetOperationMode(INT anOpMode)
    {
        FT_OP_MODE eOpMode = FT_OP_MODE(anOpMode);
        UINT16 usValue = 0;

        switch (eOpMode)
        {
        case ADC_VALUE: usValue = 8; break;
        case BIAS_FLASH_SAVE: usValue = 9; break;
        case BIAS_NO_SAVE: usValue = 6; break;
        default: usValue = 0; break;
        }
        m_stSlaveParams.stOutPDOs.usValue1 = usValue;
    }

    inline virtual void WriteToSlave()
    {
        CEcatSlaveBase::WritePdoU(m_stSlaveParams.GET_OFFSET(Value1), m_stSlaveParams.stOutPDOs.usValue1, 16);
        CEcatSlaveBase::WritePdoU(m_stSlaveParams.GET_OFFSET(Value2), m_stSlaveParams.stOutPDOs.usValue2, 16);
    }

    inline virtual void ReadFromSlave()
    {
        m_stSlaveParams.stInPDOs.nFx = CEcatSlaveBase::ReadPdoS32(m_stSlaveParams.GET_OFFSET(Fx));
        m_stSlaveParams.stInPDOs.nFy = CEcatSlaveBase::ReadPdoS32(m_stSlaveParams.GET_OFFSET(Fy));
        m_stSlaveParams.stInPDOs.nFz = CEcatSlaveBase::ReadPdoS32(m_stSlaveParams.GET_OFFSET(Fz));
        m_stSlaveParams.stInPDOs.nTx = CEcatSlaveBase::ReadPdoS32(m_stSlaveParams.GET_OFFSET(Tx));
        m_stSlaveParams.stInPDOs.nTy = CEcatSlaveBase::ReadPdoS32(m_stSlaveParams.GET_OFFSET(Ty));
        m_stSlaveParams.stInPDOs.nTz = CEcatSlaveBase::ReadPdoS32(m_stSlaveParams.GET_OFFSET(Tz));
        m_stSlaveParams.stInPDOs.nCount = CEcatSlaveBase::ReadPdoS32(m_stSlaveParams.GET_OFFSET(Count));
    }
    inline INT32 GetFx() { return m_stSlaveParams.stInPDOs.nFx; }
    inline INT32 GetFy() { return m_stSlaveParams.stInPDOs.nFy; }
    inline INT32 GetFz() { return m_stSlaveParams.stInPDOs.nFz; }
    inline INT32 GetTx() { return m_stSlaveParams.stInPDOs.nTx; }
    inline INT32 GetTy() { return m_stSlaveParams.stInPDOs.nTy; }
    inline INT32 GetTz() { return m_stSlaveParams.stInPDOs.nTz; }
    inline INT32 GetCount() { return m_stSlaveParams.stInPDOs.nCount; }

    inline UINT16 GetValue1() { return m_stSlaveParams.stOutPDOs.usValue1; }
    inline UINT16 GetValue2() { return m_stSlaveParams.stOutPDOs.usValue2; }

private:
    struct KIST_FT_OUT
    {
        UINT16 usValue1;
        UINT16 usValue2;
    };

    struct KIST_FT_IN
    {
        INT32 nFx;
        INT32 nFy;
        INT32 nFz;
        INT32 nTx;
        INT32 nTy;
        INT32 nTz;
        INT32 nCount;
    };

    struct KIST_FT_SLAVE_PARAMS
    {
        KIST_FT_OUT stOutPDOs;
        KIST_FT_IN  stInPDOs;

        SET_OFFSET(Value1);
        SET_OFFSET(Value2);
        SET_OFFSET(Fx);
        SET_OFFSET(Fy);
        SET_OFFSET(Fz);
        SET_OFFSET(Tx);
        SET_OFFSET(Ty);
        SET_OFFSET(Tz);
        SET_OFFSET(Count);

        SET_BITPOS(Value1);
        SET_BITPOS(Value2);
        SET_BITPOS(Fx);
        SET_BITPOS(Fy);
        SET_BITPOS(Fz);
        SET_BITPOS(Tx);
        SET_BITPOS(Ty);
        SET_BITPOS(Tz);
        SET_BITPOS(Count);
    };

protected:
    virtual BOOL RegisterPDO()
    {
        /* Output PDOs */
        if (0 > (m_stSlaveParams.GET_OFFSET(Value1) = CEcatSlaveBase::RegisterPDOEntry(0x7000, 0x01, &m_stSlaveParams.GET_BITPOS(Value1), eOutput)))
            return FALSE;
        if (0 > (m_stSlaveParams.GET_OFFSET(Value2) = CEcatSlaveBase::RegisterPDOEntry(0x7000, 0x02, &m_stSlaveParams.GET_BITPOS(Value2), eOutput)))
            return FALSE;

        /* Inout PDOs */
        if (0 > (m_stSlaveParams.GET_OFFSET(Fx) = CEcatSlaveBase::RegisterPDOEntry(0x6000, 0x01, &m_stSlaveParams.GET_BITPOS(Fx), eInput)))
            return FALSE;
        if (0 > (m_stSlaveParams.GET_OFFSET(Fy) = CEcatSlaveBase::RegisterPDOEntry(0x6000, 0x02, &m_stSlaveParams.GET_BITPOS(Fy), eInput)))
            return FALSE;
        if (0 > (m_stSlaveParams.GET_OFFSET(Fz) = CEcatSlaveBase::RegisterPDOEntry(0x6000, 0x03, &m_stSlaveParams.GET_BITPOS(Fz), eInput)))
            return FALSE;
        if (0 > (m_stSlaveParams.GET_OFFSET(Fx) = CEcatSlaveBase::RegisterPDOEntry(0x6000, 0x04, &m_stSlaveParams.GET_BITPOS(Tx), eInput)))
            return FALSE;
        if (0 > (m_stSlaveParams.GET_OFFSET(Fy) = CEcatSlaveBase::RegisterPDOEntry(0x6000, 0x05, &m_stSlaveParams.GET_BITPOS(Ty), eInput)))
            return FALSE;
        if (0 > (m_stSlaveParams.GET_OFFSET(Fz) = CEcatSlaveBase::RegisterPDOEntry(0x6000, 0x06, &m_stSlaveParams.GET_BITPOS(Tz), eInput)))
            return FALSE;
        if (0 > (m_stSlaveParams.GET_OFFSET(Count) = CEcatSlaveBase::RegisterPDOEntry(0x6000, 0x06, &m_stSlaveParams.GET_BITPOS(Count), eInput)))
            return FALSE;

        return TRUE;
    }


private:
    KIST_FT_SLAVE_PARAMS m_stSlaveParams;



}; //CSlaveKistFT
#endif // _ECAT_SLAVE_KIST_FT_
