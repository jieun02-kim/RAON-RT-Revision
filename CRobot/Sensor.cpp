/*****************************************************************************
*	Name: Sensor.cpp
*	Author: Raimarius Delgado
*	Affiliation: RAIMLAB @ Myongji University
*	Description: Implementation for the CSensor base class
*	Copyright: RAIMLAB (2025)
*****************************************************************************/

#include "Sensor.h"
#include "posix_rt.h"
#include <cmath>

CSensor::CSensor(eSensorCommType aeComm, eSensorType aeType, BOOL abEnabled)
    : m_usSensorID(0)
    , m_usSensorAlias(0)
    , m_usSensorPos(0)
    , m_bEnabled(abEnabled)
    , m_strName(TSTRING("Unknown"))
{
    // Allocate memory for structures
    m_pstSensorInfo = new ST_SENSOR_INFO();
    m_pstSensorLimits = new ST_SENSOR_LIMITS();
    
    // Initialize sensor info
    m_pstSensorInfo->eComm = aeComm;
    m_pstSensorInfo->eType = aeType;
    m_pstSensorInfo->eState = eSensorUnknownState;
    m_pstSensorInfo->eCalibration = eSensorCalibrationNone;
    m_pstSensorInfo->uSampleRate = 1000;
    m_pstSensorInfo->uFilterCutoff = 100;
    m_pstSensorInfo->bIsCalibrated = FALSE;
    
    // Initialize status
    m_stSensorStatus.bOnline = FALSE;
    m_stSensorStatus.bOverload = FALSE;
    m_stSensorStatus.bWarning = FALSE;
    m_stSensorStatus.bCalibrating = FALSE;
    m_stSensorStatus.uErrorCode = 0;
    m_stSensorStatus.uDataCount = 0;
    m_stSensorStatus.uLastUpdateTime = 0;
}

CSensor::~CSensor()
{
    if (m_pstSensorInfo) {
        delete m_pstSensorInfo;
        m_pstSensorInfo = nullptr;
    }
    
    if (m_pstSensorLimits) {
        delete m_pstSensorLimits;
        m_pstSensorLimits = nullptr;
    }
}

BOOL CSensor::InitHW()
{
    // Default hardware initialization
    // Override in derived classes for specific hardware
    return TRUE;
}

BOOL CSensor::InitSW()
{
    // Default software initialization
    SetState(eSensorInit);
    return TRUE;
}

BOOL CSensor::DeInitHW()
{
    // Default hardware deinitialization
    return TRUE;
}

BOOL CSensor::DeInitSW()
{
    // Default software deinitialization
    SetState(eSensorDeinit);
    m_stSensorStatus.bOnline = FALSE;
    return TRUE;
}

BOOL CSensor::SetLimits(double adMin, double adMax, double adOverload, double adWarning)
{
    if (!ValidateRange(adMin, adMax)) {
        DBG_LOG_ERROR("(%s) Invalid limit range: Min=%f, Max=%f", "CSensor", adMin, adMax);
        return FALSE;
    }
    
    m_pstSensorLimits->dMinValue = adMin;
    m_pstSensorLimits->dMaxValue = adMax;
    m_pstSensorLimits->dOverloadThreshold = adOverload;
    m_pstSensorLimits->dWarningThreshold = adWarning;
    m_pstSensorLimits->bIsSet = TRUE;
    
    DBG_LOG_INFO("(%s) Sensor[%d] Limits set: Min=%f, Max=%f, Overload=%f, Warning=%f", 
                 "CSensor", m_usSensorID, adMin, adMax, adOverload, adWarning);
    
    return TRUE;
}

void CSensor::SetState(eSensorState aeState)
{
    if (m_pstSensorInfo->eState != aeState) {
        eSensorState ePrevState = m_pstSensorInfo->eState;
        m_pstSensorInfo->eState = aeState;
        
        DBG_LOG_INFO("(%s) Sensor[%d] State changed: %s -> %s", 
                     "CSensor", m_usSensorID, 
                     GetSensorStateString(ePrevState).c_str(),
                     GetSensorStateString(aeState).c_str());
        
        // Update status based on state
        switch (aeState) {
            case eSensorRunning:
                m_stSensorStatus.bOnline = TRUE;
                break;
            case eSensorError:
            case eSensorFault:
            case eSensorDisconnected:
                m_stSensorStatus.bOnline = FALSE;
                break;
            case eSensorCalibrating:
                m_stSensorStatus.bCalibrating = TRUE;
                break;
            case eSensorIdle:
                m_stSensorStatus.bCalibrating = FALSE;
                break;
            default:
                break;
        }
    }
}

BOOL CSensor::CheckLimits(double adValue)
{
    if (!m_pstSensorLimits->bIsSet) {
        return TRUE; // No limits set
    }
    
    // Check overload threshold
    if (m_pstSensorLimits->dOverloadThreshold > 0.0) {
        if (fabs(adValue) > m_pstSensorLimits->dOverloadThreshold) {
            m_stSensorStatus.bOverload = TRUE;
            DBG_LOG_WARN("(%s) Sensor[%d] Overload detected: Value=%f, Threshold=%f", 
                         "CSensor", m_usSensorID, adValue, m_pstSensorLimits->dOverloadThreshold);
            return FALSE;
        }
    }
    
    // Check warning threshold
    if (m_pstSensorLimits->dWarningThreshold > 0.0) {
        if (fabs(adValue) > m_pstSensorLimits->dWarningThreshold) {
            m_stSensorStatus.bWarning = TRUE;
            DBG_LOG_WARN("(%s) Sensor[%d] Warning threshold exceeded: Value=%f, Threshold=%f", 
                         "CSensor", m_usSensorID, adValue, m_pstSensorLimits->dWarningThreshold);
        }
    }
    
    return IsValueInRange(adValue);
}

BOOL CSensor::IsValueInRange(double adValue)
{
    if (!m_pstSensorLimits->bIsSet) {
        return TRUE; // No limits set
    }
    
    return (adValue >= m_pstSensorLimits->dMinValue && adValue <= m_pstSensorLimits->dMaxValue);
}

void CSensor::UpdateStatus()
{
    // Update timestamp
    m_stSensorStatus.uLastUpdateTime = read_timer();
    
    // Check overload conditions
    CheckOverload();
    
    // Clear warning if no longer applicable
    if (m_stSensorStatus.bWarning) {
        // This should be implemented by derived classes based on specific conditions
        // For now, just log it
        DBG_LOG_WARN("(%s) Sensor[%d] Warning status active", "CSensor", m_usSensorID);
    }
}

void CSensor::CheckOverload()
{
    // Base implementation - override in derived classes
    // This is called from UpdateStatus() and should implement sensor-specific overload detection
    
    if (m_stSensorStatus.bOverload) {
        SetState(eSensorError);
        SetErrorCode(0x01); // Generic overload error
    }
}

TSTRING CSensor::GetSensorTypeString() const
{
    switch (m_pstSensorInfo->eType) {
        case eSensorForceTorque:    return TSTRING("Force/Torque");
        case eSensorVision:         return TSTRING("Vision");
        case eSensorProximity:      return TSTRING("Proximity");
        case eSensorTemperature:    return TSTRING("Temperature");
        case eSensorPressure:       return TSTRING("Pressure");
        case eSensorAccelerometer:  return TSTRING("Accelerometer");
        case eSensorGyroscope:      return TSTRING("Gyroscope");
        case eSensorIMU:            return TSTRING("IMU");
        case eSensorLidar:          return TSTRING("Lidar");
        case eSensorEncoder:        return TSTRING("Encoder");
        case eSensorTouch:          return TSTRING("Touch");
        case eSensorUnknown:
        default:                    return TSTRING("Unknown");
    }
}

TSTRING CSensor::GetSensorStateString() const
{
    return GetSensorStateString(m_pstSensorInfo->eState);
}

TSTRING CSensor::GetSensorStateString(eSensorState aeState) const
{
    switch (aeState) {
        case eSensorEmergency:      return TSTRING("Emergency");
        case eSensorError:          return TSTRING("Error");
        case eSensorUnknownState:   return TSTRING("Unknown");
        case eSensorDeinit:         return TSTRING("Deinitialized");
        case eSensorDisconnected:   return TSTRING("Disconnected");
        case eSensorInit:           return TSTRING("Initialized");
        case eSensorIdle:           return TSTRING("Idle");
        case eSensorCalibrating:    return TSTRING("Calibrating");
        case eSensorRunning:        return TSTRING("Running");
        case eSensorStopped:        return TSTRING("Stopped");
        case eSensorFault:          return TSTRING("Fault");
        default:                    return TSTRING("Unknown");
    }
}

BOOL CSensor::ValidateRange(double adMin, double adMax)
{
    return (adMin < adMax);
}
