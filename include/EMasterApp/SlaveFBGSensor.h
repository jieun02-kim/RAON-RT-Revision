/*****************************************************************************
*	Name: SlaveKistFT.h
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Header for the CSlaveKistFT child class.
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/

#ifndef _ECAT_SLAVE_LAN9252_FBG_
#define _ECAT_SLAVE_LAN9252_FBG_

#include "EcatSlaveBase.h"

#ifndef LAN9252_VENDOR_ID
#define LAN9252_VENDOR_ID         		0xe00004d8
#endif
#define LAN9252_FBG_SENSOR 				0x00009252
#define KIST_FT_ACTIVATE_WORD       0x0300
#define KIST_FT_SYNC0_SHIFT         125000

/* Master 0, Slave 0, "LAN9252_t"
* Vendor ID:       0xe00004d8
* Product code:    0x00009252
* Revision number: 0x00000001
*/
ec_pdo_entry_info_t slave_0_pdo_entries[] = {
    {0x0005, 0x01, 32}, /* Theta */
    {0x0005, 0x02, 8}, /* Biasing */
    {0x0006, 0x01, 32}, /* Fx_L */
    {0x0006, 0x02, 32}, /* Fy_L */
    {0x0006, 0x03, 32}, /* Fz_L */
    {0x0006, 0x04, 32}, /* Temp_L */
    {0x0006, 0x05, 32}, /* Fx_R */
    {0x0006, 0x06, 32}, /* Fy_R */
    {0x0006, 0x07, 32}, /* Fz_R */
    {0x0006, 0x08, 32}, /* Temp_R */
    {0x0006, 0x09, 32}, /* Fx */
    {0x0006, 0x0a, 32}, /* Fy */
    {0x0006, 0x0b, 32}, /* Fz */
    {0x0006, 0x0c, 32}, /* Fg */
    {0x0006, 0x0d, 32}, /* Tz */
    {0x0006, 0x0e, 32}, /* Temp */
};

ec_pdo_info_t slave_0_pdos[] = {
    {0x1600, 2, slave_0_pdo_entries + 0}, /* Outputs */
    {0x1a00, 14, slave_0_pdo_entries + 2}, /* Inputs */
};

ec_sync_info_t slave_0_syncs[] = {
    {0, EC_DIR_OUTPUT, 1, slave_0_pdos + 0, EC_WD_ENABLE},
    {1, EC_DIR_INPUT, 1, slave_0_pdos + 1, EC_WD_DISABLE},
    {0xff}
};

class CSlaveFBGSensor : public CEcatSlaveBase
{
public:
    CSlaveFBGSensor()
    {
        CEcatSlaveBase::SetVendorID((UINT32)LAN9252_VENDOR_ID);
        CEcatSlaveBase::SetProductCode((UINT32)LAN9252_FBG_SENSOR);
        CEcatSlaveBase::SetDeviceType(eFTSensor);
        CEcatSlaveBase::SetDCSupported(FALSE);
        //CEcatSlaveBase::SetDCActivateWord(KIST_FT_ACTIVATE_WORD);
        //CEcatSlaveBase::SetDCShiftTime((INT32)KIST_FT_SYNC0_SHIFT);
        CEcatSlaveBase::SetEcatPdoSync(slave_0_syncs);
    };
    ~CSlaveFBGSensor() {};


public:
    inline void SetBiasing(BOOL abEnable)
    {
        m_stSlaveParams.stOutPDOs.btBias = abEnable;
    }

    inline void SetTheta(INT32 anTheta)
    {
        m_stSlaveParams.stOutPDOs.nTheta = anTheta;
    }

    inline virtual void WriteToSlave()
    {
        CEcatSlaveBase::WritePdoS(m_stSlaveParams.GET_OFFSET(Theta), m_stSlaveParams.stOutPDOs.nTheta, 32);
        CEcatSlaveBase::WritePdoU(m_stSlaveParams.GET_OFFSET(Bias), m_stSlaveParams.stOutPDOs.btBias, 8);
    }

    inline virtual void ReadFromSlave()
    {
        m_stSlaveParams.stInPDOs.nLeftFx = CEcatSlaveBase::ReadPdoS32(m_stSlaveParams.GET_OFFSET(LeftFx));
        m_stSlaveParams.stInPDOs.nLeftFy = CEcatSlaveBase::ReadPdoS32(m_stSlaveParams.GET_OFFSET(LeftFy));
        m_stSlaveParams.stInPDOs.nLeftFz = CEcatSlaveBase::ReadPdoS32(m_stSlaveParams.GET_OFFSET(LeftFz));
        m_stSlaveParams.stInPDOs.nLeftTemp = CEcatSlaveBase::ReadPdoS32(m_stSlaveParams.GET_OFFSET(LeftTemp));

        m_stSlaveParams.stInPDOs.nRightFx = CEcatSlaveBase::ReadPdoS32(m_stSlaveParams.GET_OFFSET(RightFx));
        m_stSlaveParams.stInPDOs.nRightFy = CEcatSlaveBase::ReadPdoS32(m_stSlaveParams.GET_OFFSET(RightFy));
        m_stSlaveParams.stInPDOs.nRightFz = CEcatSlaveBase::ReadPdoS32(m_stSlaveParams.GET_OFFSET(RightFz));
        m_stSlaveParams.stInPDOs.nRightTemp = CEcatSlaveBase::ReadPdoS32(m_stSlaveParams.GET_OFFSET(RightTemp));

        m_stSlaveParams.stInPDOs.nFx = CEcatSlaveBase::ReadPdoS32(m_stSlaveParams.GET_OFFSET(Fx));
        m_stSlaveParams.stInPDOs.nFy = CEcatSlaveBase::ReadPdoS32(m_stSlaveParams.GET_OFFSET(Fy));
        m_stSlaveParams.stInPDOs.nFz = CEcatSlaveBase::ReadPdoS32(m_stSlaveParams.GET_OFFSET(Fz));
        m_stSlaveParams.stInPDOs.nFg = CEcatSlaveBase::ReadPdoS32(m_stSlaveParams.GET_OFFSET(Fg));
        m_stSlaveParams.stInPDOs.nTz = CEcatSlaveBase::ReadPdoS32(m_stSlaveParams.GET_OFFSET(Tz));
        m_stSlaveParams.stInPDOs.nTemp = CEcatSlaveBase::ReadPdoS32(m_stSlaveParams.GET_OFFSET(Temp));
    }

    inline INT32 GetLeftFx() { return m_stSlaveParams.stInPDOs.nLeftFx; }
    inline INT32 GetLeftFy() { return m_stSlaveParams.stInPDOs.nLeftFy; }
    inline INT32 GetLeftFz() { return m_stSlaveParams.stInPDOs.nLeftFz; }
    inline INT32 GetLeftTemp() { return m_stSlaveParams.stInPDOs.nLeftTemp; }
    
    inline INT32 GetRightFx() { return m_stSlaveParams.stInPDOs.nRightFx; }
    inline INT32 GetRightFy() { return m_stSlaveParams.stInPDOs.nRightFy; }
    inline INT32 GetRightFz() { return m_stSlaveParams.stInPDOs.nRightFz; }
    inline INT32 GetRightTemp() { return m_stSlaveParams.stInPDOs.nRightTemp; }
    
    inline INT32 GetFx() { return m_stSlaveParams.stInPDOs.nFx; }
    inline INT32 GetFy() { return m_stSlaveParams.stInPDOs.nFy; }
    inline INT32 GetFz() { return m_stSlaveParams.stInPDOs.nFz; }
    inline INT32 GetFg() { return m_stSlaveParams.stInPDOs.nFg; }
    inline INT32 GetTz() { return m_stSlaveParams.stInPDOs.nTz; }
    inline INT32 GetTemp() { return m_stSlaveParams.stInPDOs.nTemp; }

    inline INT32 GetTheta() { return m_stSlaveParams.stOutPDOs.nTheta; }
    inline BOOL GetBias() { return m_stSlaveParams.stOutPDOs.btBias; }

private:
    struct FBG_SENSOR_OUT
    {
        INT32 nTheta;
        UINT8 btBias;
    };

    struct FBG_SENSOR_IN
    {
        INT32 nLeftFx;
        INT32 nLeftFy;
        INT32 nLeftFz;
        INT32 nLeftTemp;

        INT32 nRightFx;
        INT32 nRightFy;
        INT32 nRightFz;
        INT32 nRightTemp;

        INT32 nFx;
        INT32 nFy;
        INT32 nFz;
        INT32 nFg;
        INT32 nTz;

        INT32 nTemp;
    };

    struct FBG_SENSOR_SLAVE_PARAMS
    {
        FBG_SENSOR_OUT stOutPDOs;
        FBG_SENSOR_IN  stInPDOs;

        SET_OFFSET(Theta);
        SET_OFFSET(Bias);
        SET_OFFSET(LeftFx);
        SET_OFFSET(LeftFy);
        SET_OFFSET(LeftFz);
        SET_OFFSET(LeftTemp);
        SET_OFFSET(RightFx);
        SET_OFFSET(RightFy);
        SET_OFFSET(RightFz);
        SET_OFFSET(RightTemp);
        SET_OFFSET(Fx);
        SET_OFFSET(Fy);
        SET_OFFSET(Fz);
        SET_OFFSET(Fg);
        SET_OFFSET(Tz);
        SET_OFFSET(Temp);

        SET_BITPOS(Theta);
        SET_BITPOS(Bias);
        SET_BITPOS(LeftFx);
        SET_BITPOS(LeftFy);
        SET_BITPOS(LeftFz);
        SET_BITPOS(LeftTemp);
        SET_BITPOS(RightFx);
        SET_BITPOS(RightFy);
        SET_BITPOS(RightFz);
        SET_BITPOS(RightTemp);
        SET_BITPOS(Fx);
        SET_BITPOS(Fy);
        SET_BITPOS(Fz);
        SET_BITPOS(Fg);
        SET_BITPOS(Tz);
        SET_BITPOS(Temp);
    };

protected:
    virtual BOOL RegisterPDO()
    {
        /* Output PDOs */
        if (0 > (m_stSlaveParams.GET_OFFSET(Theta) = CEcatSlaveBase::RegisterPDOEntry(0x0005, 0x01, &m_stSlaveParams.GET_BITPOS(Theta), eOutput)))
            return FALSE;
        if (0 > (m_stSlaveParams.GET_OFFSET(Bias) = CEcatSlaveBase::RegisterPDOEntry(0x0005, 0x02, &m_stSlaveParams.GET_BITPOS(Bias), eOutput)))
            return FALSE;

        /* Inout PDOs */
        if (0 > (m_stSlaveParams.GET_OFFSET(LeftFx) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x01, &m_stSlaveParams.GET_BITPOS(LeftFx), eInput)))
            return FALSE;
        if (0 > (m_stSlaveParams.GET_OFFSET(LeftFy) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x02, &m_stSlaveParams.GET_BITPOS(LeftFy), eInput)))
            return FALSE;
        if (0 > (m_stSlaveParams.GET_OFFSET(LeftFz) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x03, &m_stSlaveParams.GET_BITPOS(LeftFz), eInput)))
            return FALSE;
        if (0 > (m_stSlaveParams.GET_OFFSET(LeftTemp) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x04, &m_stSlaveParams.GET_BITPOS(LeftTemp), eInput)))
            return FALSE;

        if (0 > (m_stSlaveParams.GET_OFFSET(RightFx) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x05, &m_stSlaveParams.GET_BITPOS(RightFx), eInput)))
            return FALSE;
        if (0 > (m_stSlaveParams.GET_OFFSET(RightFy) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x06, &m_stSlaveParams.GET_BITPOS(RightFy), eInput)))
            return FALSE;
        if (0 > (m_stSlaveParams.GET_OFFSET(RightFz) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x07, &m_stSlaveParams.GET_BITPOS(RightFz), eInput)))
            return FALSE;
        if (0 > (m_stSlaveParams.GET_OFFSET(RightTemp) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x08, &m_stSlaveParams.GET_BITPOS(RightTemp), eInput)))
            return FALSE;

        if (0 > (m_stSlaveParams.GET_OFFSET(Fx) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x09, &m_stSlaveParams.GET_BITPOS(Fx), eInput)))
            return FALSE;
        if (0 > (m_stSlaveParams.GET_OFFSET(Fy) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x0A, &m_stSlaveParams.GET_BITPOS(Fy), eInput)))
            return FALSE;
        if (0 > (m_stSlaveParams.GET_OFFSET(Fz) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x0B, &m_stSlaveParams.GET_BITPOS(Fz), eInput)))
            return FALSE;
        if (0 > (m_stSlaveParams.GET_OFFSET(Fg) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x0C, &m_stSlaveParams.GET_BITPOS(Fg), eInput)))
            return FALSE;
        if (0 > (m_stSlaveParams.GET_OFFSET(Tz) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x0D, &m_stSlaveParams.GET_BITPOS(Tz), eInput)))
            return FALSE;
        if (0 > (m_stSlaveParams.GET_OFFSET(Temp) = CEcatSlaveBase::RegisterPDOEntry(0x0006, 0x0E, &m_stSlaveParams.GET_BITPOS(Temp), eInput)))
            return FALSE;


        return TRUE;
    }


private:
    FBG_SENSOR_SLAVE_PARAMS m_stSlaveParams;



}; //CSlaveKistFT
#endif // _ECAT_SLAVE_KIST_FT_
