/*****************************************************************************
*	Name: SlaveNRMKEndTool.cpp
*	Author: Raimarius Delgado
*	Affiliation: RAIMLAB @ Myongji University
*	Description: Implementation of the CSlaveNrmkEndtool child class.
*	Copyright: RAIMLAB (2025)
*****************************************************************************/

#include "SlaveNRMKEndTool.h"

CSlaveNrmkEndtool::CSlaveNrmkEndtool()
{
    // Initialize slave parameters
    memset(&m_stSlaveParams, 0, sizeof(m_stSlaveParams));
    
    // Set vendor/product info
    SetVendorInfo(NRMK_VENDOR_ID, NRMK_ENDTOOL);
    SetDCInfo(TRUE, NRMK_ENDTOOL_ACTIVATE_WORD, NRMK_ENDTOOL_SYNC0_SHIFT);
    
    // Set device type
    SetDeviceType(eFTSensor);
    
    // Initialize FT sensor state
    m_bIsFTOn = false;
    m_bFTOn = false;
    m_bFTBiasing = false;
    m_nFTOutputRate = 1000;
    m_nFTLPFCof = 100;
    m_eFTState = FT_NONE;
}

CSlaveNrmkEndtool::~CSlaveNrmkEndtool()
{
}

BOOL CSlaveNrmkEndtool::RegisterPDO()
{
    // Register Output PDOs (to slave)
    if (0 > (m_stSlaveParams.GET_OFFSET(ILed) = RegisterPDOEntry(0x7000, 0x01, &m_stSlaveParams.GET_BITPOS(ILed), eOutput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(IGripper) = RegisterPDOEntry(0x7000, 0x02, &m_stSlaveParams.GET_BITPOS(IGripper), eOutput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(FTConfigParam) = RegisterPDOEntry(0x7000, 0x03, &m_stSlaveParams.GET_BITPOS(FTConfigParam), eOutput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(LEDMode) = RegisterPDOEntry(0x7000, 0x04, &m_stSlaveParams.GET_BITPOS(LEDMode), eOutput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(LEDG) = RegisterPDOEntry(0x7000, 0x05, &m_stSlaveParams.GET_BITPOS(LEDG), eOutput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(LEDR) = RegisterPDOEntry(0x7000, 0x06, &m_stSlaveParams.GET_BITPOS(LEDR), eOutput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(LEDB) = RegisterPDOEntry(0x7000, 0x07, &m_stSlaveParams.GET_BITPOS(LEDB), eOutput)))
        return FALSE;

    // Register Input PDOs (from slave)
    if (0 > (m_stSlaveParams.GET_OFFSET(IStatus) = RegisterPDOEntry(0x6000, 0x01, &m_stSlaveParams.GET_BITPOS(IStatus), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(IButton) = RegisterPDOEntry(0x6000, 0x02, &m_stSlaveParams.GET_BITPOS(IButton), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(FTRawFx) = RegisterPDOEntry(0x6000, 0x03, &m_stSlaveParams.GET_BITPOS(FTRawFx), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(FTRawFy) = RegisterPDOEntry(0x6000, 0x04, &m_stSlaveParams.GET_BITPOS(FTRawFy), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(FTRawFz) = RegisterPDOEntry(0x6000, 0x05, &m_stSlaveParams.GET_BITPOS(FTRawFz), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(FTRawTx) = RegisterPDOEntry(0x6000, 0x06, &m_stSlaveParams.GET_BITPOS(FTRawTx), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(FTRawTy) = RegisterPDOEntry(0x6000, 0x07, &m_stSlaveParams.GET_BITPOS(FTRawTy), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(FTRawTz) = RegisterPDOEntry(0x6000, 0x08, &m_stSlaveParams.GET_BITPOS(FTRawTz), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(FTOverloadStatus) = RegisterPDOEntry(0x6000, 0x09, &m_stSlaveParams.GET_BITPOS(FTOverloadStatus), eInput)))
        return FALSE;
    if (0 > (m_stSlaveParams.GET_OFFSET(FTErrorFlag) = RegisterPDOEntry(0x6000, 0x0A, &m_stSlaveParams.GET_BITPOS(FTErrorFlag), eInput)))
        return FALSE;

    return TRUE;
}

void CSlaveNrmkEndtool::ReadFromSlave()
{
    if (!IsSlaveOnline()) return;

    // Read input PDOs
    m_stSlaveParams.stInPDOs.IStatus = ReadPdoU8(m_stSlaveParams.GET_OFFSET(IStatus));
    m_stSlaveParams.stInPDOs.IButton = ReadPdoU8(m_stSlaveParams.GET_OFFSET(IButton));
    m_stSlaveParams.stInPDOs.FTRawFx = ReadPdoS16(m_stSlaveParams.GET_OFFSET(FTRawFx));
    m_stSlaveParams.stInPDOs.FTRawFy = ReadPdoS16(m_stSlaveParams.GET_OFFSET(FTRawFy));
    m_stSlaveParams.stInPDOs.FTRawFz = ReadPdoS16(m_stSlaveParams.GET_OFFSET(FTRawFz));
    m_stSlaveParams.stInPDOs.FTRawTx = ReadPdoS16(m_stSlaveParams.GET_OFFSET(FTRawTx));
    m_stSlaveParams.stInPDOs.FTRawTy = ReadPdoS16(m_stSlaveParams.GET_OFFSET(FTRawTy));
    m_stSlaveParams.stInPDOs.FTRawTz = ReadPdoS16(m_stSlaveParams.GET_OFFSET(FTRawTz));
    m_stSlaveParams.stInPDOs.FTOverloadStatus = ReadPdoU8(m_stSlaveParams.GET_OFFSET(FTOverloadStatus));
    m_stSlaveParams.stInPDOs.FTErrorFlag = ReadPdoU8(m_stSlaveParams.GET_OFFSET(FTErrorFlag));

    // Calculate force/torque values
    m_stSlaveParams.stInPDOs.Fx = (float)m_stSlaveParams.stInPDOs.FTRawFx / FT_FORCE_DIVIDER;
    m_stSlaveParams.stInPDOs.Fy = (float)m_stSlaveParams.stInPDOs.FTRawFy / FT_FORCE_DIVIDER;
    m_stSlaveParams.stInPDOs.Fz = (float)m_stSlaveParams.stInPDOs.FTRawFz / FT_FORCE_DIVIDER;
    m_stSlaveParams.stInPDOs.Tx = (float)m_stSlaveParams.stInPDOs.FTRawTx / FT_TORQUE_DIVIDER;
    m_stSlaveParams.stInPDOs.Ty = (float)m_stSlaveParams.stInPDOs.FTRawTy / FT_TORQUE_DIVIDER;
    m_stSlaveParams.stInPDOs.Tz = (float)m_stSlaveParams.stInPDOs.FTRawTz / FT_TORQUE_DIVIDER;

    // Handle FT sensor errors
    if (m_stSlaveParams.stInPDOs.FTErrorFlag != 0) {
        DBG_LOG_WARN("FT Sensor Error: 0x%02x", m_stSlaveParams.stInPDOs.FTErrorFlag);
        m_eFTState = FT_FAULT;
    }
}

void CSlaveNrmkEndtool::WriteToSlave()
{
    if (!IsSlaveOnline()) return;

    // Handle FT sensor state machine
    if (m_bFTOn) {
        switch (m_eFTState) {
            case FT_NONE:
                m_eFTState = FT_READY;
                break;
            case FT_READY:
                m_bIsFTOn = false;
                m_stSlaveParams.stOutPDOs.FTConfigParam = FT_STOP_DATA_OUTPUT;
                m_eFTState = FT_SET_FILTER;
                break;
            case FT_SET_FILTER:
                m_stSlaveParams.stOutPDOs.FTConfigParam = GetFTLPFCmd(m_nFTLPFCof);
                m_eFTState = FT_SET_DATARATE;
                break;
            case FT_SET_DATARATE:
                m_stSlaveParams.stOutPDOs.FTConfigParam = GetFTDataRateCmd(m_nFTOutputRate);
                m_eFTState = FT_SET_START;
                break;
            case FT_SET_START:
                m_bIsFTOn = true;
                m_stSlaveParams.stOutPDOs.FTConfigParam = FT_START_DATA_OUTPUT;
                m_eFTState = FT_SET_BIAS;
                break;
            case FT_SET_BIAS:
                m_stSlaveParams.stOutPDOs.FTConfigParam = GetFTBiasCmd(m_bFTBiasing);
                break;
            case FT_FAULT:
                m_stSlaveParams.stOutPDOs.FTConfigParam = FT_STOP_DATA_OUTPUT;
                m_eFTState = FT_READY;
                break;
        }
    } else {
        m_stSlaveParams.stOutPDOs.FTConfigParam = FT_STOP_DATA_OUTPUT;
    }

    // Write output PDOs
    WritePdoU(m_stSlaveParams.GET_OFFSET(ILed), m_stSlaveParams.stOutPDOs.ILed, 8);
    WritePdoU(m_stSlaveParams.GET_OFFSET(IGripper), m_stSlaveParams.stOutPDOs.IGripper, 8);
    WritePdoS(m_stSlaveParams.GET_OFFSET(FTConfigParam), m_stSlaveParams.stOutPDOs.FTConfigParam, 32);
    WritePdoU(m_stSlaveParams.GET_OFFSET(LEDMode), m_stSlaveParams.stOutPDOs.LEDMode, 8);
    WritePdoU(m_stSlaveParams.GET_OFFSET(LEDG), m_stSlaveParams.stOutPDOs.LEDG, 8);
    WritePdoU(m_stSlaveParams.GET_OFFSET(LEDR), m_stSlaveParams.stOutPDOs.LEDR, 8);
    WritePdoU(m_stSlaveParams.GET_OFFSET(LEDB), m_stSlaveParams.stOutPDOs.LEDB, 8);
}

void CSlaveNrmkEndtool::FTSensorOn(bool biasing, int output_rate, int lpf_cof)
{
    m_bFTBiasing = biasing;
    m_nFTOutputRate = output_rate;
    m_nFTLPFCof = lpf_cof;

    if (!m_bIsFTOn) {
        m_bFTOn = true;
        m_eFTState = FT_NONE;
    }
}

void CSlaveNrmkEndtool::FTSensorOff()
{
    if (m_bIsFTOn) {
        m_bFTOn = false;
        m_bIsFTOn = false;
    }
}

// Helper methods for FT sensor configuration
UINT8 CSlaveNrmkEndtool::GetFTDataRate(int rate)
{
    switch (rate) {
        case 1000: return 0x08;
        case 500:  return 0x07;
        case 333:  return 0x06;
        case 200:  return 0x05;
        case 100:  return 0x04;
        case 50:   return 0x03;
        case 20:   return 0x02;
        case 10:   return 0x01;
        default:   return 0x00; // 200Hz default
    }
}

UINT8 CSlaveNrmkEndtool::GetFTLPFCof(int cof)
{
    switch (cof) {
        case 500: return 0x01;
        case 300: return 0x02;
        case 200: return 0x03;
        case 150: return 0x04;
        case 100: return 0x05;
        case 50:  return 0x06;
        case 40:  return 0x07;
        case 30:  return 0x08;
        case 20:  return 0x09;
        case 10:  return 0x0A;
        case 5:   return 0x0B;
        case 3:   return 0x0C;
        case 2:   return 0x0D;
        case 1:   return 0x0E;
        case 0:   return 0x00;
        default:  return 0x00;
    }
}

INT32 CSlaveNrmkEndtool::GetFTDataRateCmd(int datarate)
{
    return FT_DATARATE_COMMAND + (GetFTDataRate(datarate) << 8);
}

INT32 CSlaveNrmkEndtool::GetFTLPFCmd(int cof)
{
    return FT_FILTER_COMMAND + (GetFTLPFCof(cof) << 16);
}

INT32 CSlaveNrmkEndtool::GetFTBiasCmd(bool bias)
{
    if (bias) {
        return FT_BIAS_COMMAND + (1 << 8);
    }
    return FT_BIAS_COMMAND;
}