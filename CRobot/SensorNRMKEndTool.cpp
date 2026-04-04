/*****************************************************************************
*	Name: SensorNRMKEndTool.cpp
*	Author: Raimarius Delgado
*	Affiliation: RAIMLAB @ Myongji University
*	Description: Implementation for the CSensorNRMKEndTool class
*	Copyright: RAIMLAB (2025)
*****************************************************************************/

#include "SensorNRMKEndTool.h"
#include "posix_rt.h"

CSensorNRMKEndTool::CSensorNRMKEndTool(BOOL abEnabled)
    : CSensorFT(eSensorEtherCAT, abEnabled)
    , m_pEcMaster(nullptr)
    , m_eLEDMode(CSlaveNrmkEndtool::NO_COLOR)
    , m_bLEDBlink(FALSE)
    , m_bSlaveInitialized(FALSE)
    , m_uLastDataTime(0)
{
    SetName(TSTRING("NRMK_EndTool"));
    
    // Set NRMK-specific dividers
    SetDividers(50.0f, 2000.0f);
    
    // Set default sample rate and filter
    m_pstSensorInfo->uSampleRate = 1000;
    m_pstSensorInfo->uFilterCutoff = 100;
}

CSensorNRMKEndTool::~CSensorNRMKEndTool()
{
    DeInit();
}

BOOL CSensorNRMKEndTool::Init(CEcatMaster& apEcmaster)
{
    if (!InitSW()) {
        DBG_LOG_ERROR("(%s) Software initialization failed", "CSensorNRMKEndTool");
        return FALSE;
    }
    
    // if (!m_bSlaveInitialized) {
    //     DBG_LOG_ERROR("(%s) EtherCAT slave not initialized", "CSensorNRMKEndTool");
    //     return FALSE;
    // }
    
    SetState(eSensorIdle);
    
    /* TODO: Implement Enabled and Connected logic */
    if (TRUE == InitSlave(apEcmaster))
    {
        DBG_LOG_INFO("(%s) Sensor initialized successfully", "CSensorNRMKEndTool");
        return TRUE;
    }
    return FALSE;
}

BOOL CSensorNRMKEndTool::DeInit()
{
    if (IsRunning()) {
        Stop();
    }
    
    m_cNrmkSlave.DeInit();
    m_bSlaveInitialized = FALSE;
    
    DeInitSW();
    
    DBG_LOG_INFO("(%s) Sensor deinitialized", "CSensorNRMKEndTool");
    
    return TRUE;
}

BOOL CSensorNRMKEndTool::Start()
{
    if (!IsOnline()) {
        DBG_LOG_ERROR("(%s) Cannot start - sensor not online", "CSensorNRMKEndTool");
        return FALSE;
    }
    
    // Start FT sensor
    m_cNrmkSlave.FTSensorOn(true, m_pstSensorInfo->uSampleRate, m_pstSensorInfo->uFilterCutoff);
    
    SetState(eSensorRunning);
    
    DBG_LOG_INFO("(%s) Sensor started", "CSensorNRMKEndTool");
    
    return TRUE;
}

BOOL CSensorNRMKEndTool::Stop()
{
    if (!IsRunning()) {
        return TRUE;
    }
    
    // Stop FT sensor
    m_cNrmkSlave.FTSensorOff();
    
    SetState(eSensorStopped);
    
    DBG_LOG_INFO("(%s) Sensor stopped", "CSensorNRMKEndTool");
    
    return TRUE;
}

BOOL CSensorNRMKEndTool::Reset()
{
    if (IsRunning()) {
        Stop();
    }
    
    // Reset error states
    ClearError();
    m_stSensorStatus.bOverload = FALSE;
    m_stSensorStatus.bWarning = FALSE;
    
    // Reset LED to default
    LED_OFF();
    
    SetState(eSensorIdle);
    
    DBG_LOG_INFO("(%s) Sensor reset", "CSensorNRMKEndTool");
    
    return TRUE;
}

BOOL CSensorNRMKEndTool::Calibrate(eSensorCalibration aeCalType)
{
    if (!IsOnline()) {
        DBG_LOG_ERROR("(%s) Cannot calibrate - sensor not online", "CSensorNRMKEndTool");
        return FALSE;
    }
    
    // // Indicate calibration with LED
    // LED_YELLOW(TRUE);
    
    // BOOL bResult = CSensorFT::Calibrate(aeCalType);
    
    // if (bResult) {
    //     LED_GREEN(FALSE);
    //     Sleep(1000);
    //     LED_OFF();
    // } else {
    //     LED_RED(TRUE);
    //     Sleep(1000);
    //     LED_OFF();
    // }
    
    return TRUE;
}

BOOL CSensorNRMKEndTool::ReadData()
{
    if (!IsOnline()) {
        return FALSE;
    }
    
    // Read data from slave
    m_cNrmkSlave.ReadFromSlave();
    
    // Update raw data
    UpdateRawData(m_cNrmkSlave.GetFTRawFx(), m_cNrmkSlave.GetFTRawFy(), m_cNrmkSlave.GetFTRawFz(),
                  m_cNrmkSlave.GetFTRawTx(), m_cNrmkSlave.GetFTRawTy(), m_cNrmkSlave.GetFTRawTz());
    
    // Process the data
    ProcessRawData();
    
    // Handle errors
    HandleFTErrors();
    
    // Update LED status
    UpdateLEDStatus();
    
    IncrementDataCount();
    UpdateStatus();
    
    return TRUE;
}

BOOL CSensorNRMKEndTool::IsDataReady()
{
    if (!IsOnline()) {
        return FALSE;
    }
    
    // Check if FT sensor is providing data
    return m_cNrmkSlave.IsFTSensorOn();
}

BOOL CSensorNRMKEndTool::SetSampleRate(UINT32 auRate)
{
    if (auRate == 0) {
        DBG_LOG_ERROR("(%s) Invalid sample rate: %d", "CSensorNRMKEndTool", auRate);
        return FALSE;
    }
    
    m_pstSensorInfo->uSampleRate = auRate;
    
    // If sensor is running, restart with new rate
    if (IsRunning()) {
        Stop();
        Start();
    }
    
    DBG_LOG_INFO("(%s) Sample rate set to: %d Hz", "CSensorNRMKEndTool", auRate);
    
    return TRUE;
}

BOOL CSensorNRMKEndTool::SetFilterCutoff(UINT32 auCutoff)
{
    m_pstSensorInfo->uFilterCutoff = auCutoff;
    
    // If sensor is running, restart with new filter
    if (IsRunning()) {
        Stop();
        Start();
    }
    
    DBG_LOG_INFO("(%s) Filter cutoff set to: %d Hz", "CSensorNRMKEndTool", auCutoff);
    
    return TRUE;
}

BOOL CSensorNRMKEndTool::InitSlave(CEcatMaster& apEcmaster)
{
    if (m_bSlaveInitialized) {
        DBG_LOG_WARN("(%s) Slave already initialized", "CSensorNRMKEndTool");
        return TRUE;
    }
    
    m_pEcMaster = &apEcmaster;
    m_pEcMaster->AddSlave(&m_cNrmkSlave);
    
    DBG_LOG_INFO("(%s) EtherCAT slave initialized", "CSensorNRMKEndTool");
    
    return TRUE;
}

void CSensorNRMKEndTool::SetVendorInfo(UINT32 auVendorID, UINT32 auProductCode)
{
    m_cNrmkSlave.SetVendorInfo(auVendorID, auProductCode);
}

void CSensorNRMKEndTool::SetDCInfo(BOOL abDCSupported, UINT16 ausActivateWord, INT32 anShiftTime)
{
    m_cNrmkSlave.SetDCInfo(abDCSupported, ausActivateWord, anShiftTime);
}

void CSensorNRMKEndTool::SetLED(CSlaveNrmkEndtool::LED_MODE aeLEDMode)
{
    m_eLEDMode = aeLEDMode;
    m_cNrmkSlave.SetLEDMode((UINT8)aeLEDMode);
}

void CSensorNRMKEndTool::LED_OFF()
{
    SetLED(CSlaveNrmkEndtool::NO_COLOR);
}

void CSensorNRMKEndTool::LED_RED(BOOL abBlink)
{
    SetLED(abBlink ? CSlaveNrmkEndtool::BLINK_RED : CSlaveNrmkEndtool::STEADY_RED);
}

void CSensorNRMKEndTool::LED_GREEN(BOOL abBlink)
{
    SetLED(abBlink ? CSlaveNrmkEndtool::BLINK_GREEN : CSlaveNrmkEndtool::STEADY_GREEN);
}

void CSensorNRMKEndTool::LED_BLUE(BOOL abBlink)
{
    SetLED(abBlink ? CSlaveNrmkEndtool::BLINK_BLUE : CSlaveNrmkEndtool::STEADY_BLUE);
}

void CSensorNRMKEndTool::LED_YELLOW(BOOL abBlink)
{
    SetLED(abBlink ? CSlaveNrmkEndtool::BLINK_YELLOW : CSlaveNrmkEndtool::STEADY_YELLOW);
}

void CSensorNRMKEndTool::SetGripper(UINT8 auValue)
{
    m_cNrmkSlave.SetIGripper(auValue);
}

UINT8 CSensorNRMKEndTool::GetGripper() const
{
    return m_cNrmkSlave.GetIStatus(); // Assuming gripper status is in IStatus
}

BOOL CSensorNRMKEndTool::IsButtonPressed() const
{
    return (m_cNrmkSlave.GetIButton() != 0);
}

UINT8 CSensorNRMKEndTool::GetStatus() const
{
    return m_cNrmkSlave.GetIStatus();
}

BOOL CSensorNRMKEndTool::IsFTSensorOn() const
{
    return m_cNrmkSlave.IsFTSensorOn();
}

UINT8 CSensorNRMKEndTool::GetFTErrorFlag() const
{
    return m_cNrmkSlave.GetFTErrorFlag();
}

UINT8 CSensorNRMKEndTool::GetFTOverloadStatus() const
{
    return m_cNrmkSlave.GetFTOverloadStatus();
}

void CSensorNRMKEndTool::ProcessRawData()
{
    // Call base class processing
    CSensorFT::ProcessRawData();
    
    // Additional NRMK-specific processing can be added here
    m_uLastDataTime = read_timer();
}

void CSensorNRMKEndTool::UpdateRawData(INT16 anFx, INT16 anFy, INT16 anFz, INT16 anTx, INT16 anTy, INT16 anTz)
{
    // Call base class method
    CSensorFT::UpdateRawData(anFx, anFy, anFz, anTx, anTy, anTz);
    
    // Write data to slave
    m_cNrmkSlave.WriteToSlave();
}

void CSensorNRMKEndTool::HandleFTErrors()
{
    UINT8 uErrorFlag = GetFTErrorFlag();
    UINT8 uOverloadStatus = GetFTOverloadStatus();
    
    if (uErrorFlag != 0) {
        SetErrorCode(uErrorFlag);
        SetState(eSensorError);
        DBG_LOG_ERROR("(%s) FT Sensor error: 0x%02x", "CSensorNRMKEndTool", uErrorFlag);
    }
    
    if (uOverloadStatus != 0) {
        m_stSensorStatus.bOverload = TRUE;
        DBG_LOG_WARN("(%s) FT Sensor overload: 0x%02x", "CSensorNRMKEndTool", uOverloadStatus);
    }
}

void CSensorNRMKEndTool::UpdateLEDStatus()
{
    // Update LED based on sensor status
    if (IsInError()) {
        LED_RED(TRUE);
    } else if (IsOverloaded()) {
        LED_YELLOW(TRUE);
    } else if (IsRunning()) {
        LED_GREEN(FALSE);
    } else {
        LED_OFF();
    }
}
