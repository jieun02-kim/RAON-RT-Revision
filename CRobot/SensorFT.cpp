/*****************************************************************************
*	Name: SensorFT.cpp
*	Author: Raimarius Delgado
*	Affiliation: RAIMLAB @ Myongji University
*	Description: Implementation for the CSensorFT class
*	Copyright: RAIMLAB (2025)
*****************************************************************************/

#include "SensorFT.h"
#include "posix_rt.h"
#include <cmath>
#include <fstream>

CSensorFT::CSensorFT(eSensorCommType aeComm, BOOL abEnabled)
    : CSensor(aeComm, eSensorForceTorque, abEnabled)
    , m_bForceOverload(FALSE)
    , m_bTorqueOverload(FALSE)
{
    // Initialize FT data
    memset(&m_stFTData, 0, sizeof(m_stFTData));
    
    // Initialize calibration with default values
    m_stFTCalibration.fForceDivider = 50.0f;   // NRMK default
    m_stFTCalibration.fTorqueDivider = 2000.0f; // NRMK default
    m_stFTCalibration.bIsValid = FALSE;
    
    // Set default limits for FT sensor
    SetLimits(-1000.0, 1000.0, 800.0, 600.0); // Example limits in Newtons
}

CSensorFT::~CSensorFT()
{
}

BOOL CSensorFT::Calibrate(eSensorCalibration aeCalType)
{
    if (!IsOnline()) {
        DBG_LOG_ERROR("(%s) Cannot calibrate - sensor not online", "CSensorFT");
        return FALSE;
    }
    
    SetState(eSensorCalibrating);
    
    BOOL bResult = FALSE;
    
    switch (aeCalType) {
        case eSensorCalibrationZero:
            bResult = ZeroBias();
            break;
        case eSensorCalibrationBias:
            bResult = ZeroBias(); // Same as zero for now
            break;
        case eSensorCalibrationScale:
            // Scale calibration would require known weights/forces
            DBG_LOG_WARN("(%s) Scale calibration not implemented", "CSensorFT");
            bResult = FALSE;
            break;
        case eSensorCalibrationFull:
            bResult = ZeroBias(); // Full calibration not implemented
            break;
        default:
            DBG_LOG_ERROR("(%s) Unknown calibration type: %d", "CSensorFT", aeCalType);
            bResult = FALSE;
            break;
    }
    
    if (bResult) {
        m_pstSensorInfo->bIsCalibrated = TRUE;
        m_pstSensorInfo->eCalibration = aeCalType;
        SetState(eSensorIdle);
        DBG_LOG_INFO("(%s) Calibration completed successfully", "CSensorFT");
    } else {
        SetState(eSensorError);
        SetErrorCode(0x02); // Calibration error
        DBG_LOG_ERROR("(%s) Calibration failed", "CSensorFT");
    }
    
    return bResult;
}

BOOL CSensorFT::ReadData()
{
    if (!IsOnline()) {
        return FALSE;
    }
    
    // This should be overridden by derived classes to read actual sensor data
    // For now, just process the existing raw data
    ProcessRawData();
    
    IncrementDataCount();
    UpdateStatus();
    
    return TRUE;
}

BOOL CSensorFT::IsDataReady()
{
    // This should be overridden by derived classes
    // For now, just check if sensor is online
    return IsOnline();
}

BOOL CSensorFT::ZeroBias()
{
    if (!IsOnline()) {
        return FALSE;
    }
    
    // Take multiple samples for bias calculation
    const int BIAS_SAMPLES = 100;
    float fSumFx = 0.0f, fSumFy = 0.0f, fSumFz = 0.0f;
    float fSumTx = 0.0f, fSumTy = 0.0f, fSumTz = 0.0f;
    
    for (int i = 0; i < BIAS_SAMPLES; i++) {
        // Read current data (this should be implemented by derived class)
        ProcessRawData();
        
        fSumFx += m_stFTData.fFx;
        fSumFy += m_stFTData.fFy;
        fSumFz += m_stFTData.fFz;
        fSumTx += m_stFTData.fTx;
        fSumTy += m_stFTData.fTy;
        fSumTz += m_stFTData.fTz;
        
    }
    
    // Calculate average bias
    m_stFTCalibration.fBiasFx = fSumFx / BIAS_SAMPLES;
    m_stFTCalibration.fBiasFy = fSumFy / BIAS_SAMPLES;
    m_stFTCalibration.fBiasFz = fSumFz / BIAS_SAMPLES;
    m_stFTCalibration.fBiasTx = fSumTx / BIAS_SAMPLES;
    m_stFTCalibration.fBiasTy = fSumTy / BIAS_SAMPLES;
    m_stFTCalibration.fBiasTz = fSumTz / BIAS_SAMPLES;
    
    m_stFTCalibration.bIsValid = TRUE;
    
    DBG_LOG_INFO("(%s) Zero bias calibration completed - Bias: Fx=%f, Fy=%f, Fz=%f, Tx=%f, Ty=%f, Tz=%f", 
                 "CSensorFT", 
                 m_stFTCalibration.fBiasFx, m_stFTCalibration.fBiasFy, m_stFTCalibration.fBiasFz,
                 m_stFTCalibration.fBiasTx, m_stFTCalibration.fBiasTy, m_stFTCalibration.fBiasTz);
    
    return TRUE;
}

BOOL CSensorFT::SetBias(float afFx, float afFy, float afFz, float afTx, float afTy, float afTz)
{
    m_stFTCalibration.fBiasFx = afFx;
    m_stFTCalibration.fBiasFy = afFy;
    m_stFTCalibration.fBiasFz = afFz;
    m_stFTCalibration.fBiasTx = afTx;
    m_stFTCalibration.fBiasTy = afTy;
    m_stFTCalibration.fBiasTz = afTz;
    
    m_stFTCalibration.bIsValid = TRUE;
    
    return TRUE;
}

BOOL CSensorFT::SetScale(float afFx, float afFy, float afFz, float afTx, float afTy, float afTz)
{
    m_stFTCalibration.fScaleFx = afFx;
    m_stFTCalibration.fScaleFy = afFy;
    m_stFTCalibration.fScaleFz = afFz;
    m_stFTCalibration.fScaleTx = afTx;
    m_stFTCalibration.fScaleTy = afTy;
    m_stFTCalibration.fScaleTz = afTz;
    
    return TRUE;
}

BOOL CSensorFT::SetDividers(float afForceDivider, float afTorqueDivider)
{
    if (afForceDivider <= 0.0f || afTorqueDivider <= 0.0f) {
        DBG_LOG_ERROR("(%s) Invalid divider values: Force=%f, Torque=%f", "CSensorFT", afForceDivider, afTorqueDivider);
        return FALSE;
    }
    
    m_stFTCalibration.fForceDivider = afForceDivider;
    m_stFTCalibration.fTorqueDivider = afTorqueDivider;
    
    DBG_LOG_INFO("(%s) Dividers set: Force=%f, Torque=%f", "CSensorFT", afForceDivider, afTorqueDivider);
    
    return TRUE;
}

void CSensorFT::GetForce(float& afFx, float& afFy, float& afFz) const
{
    afFx = m_stFTData.fFx;
    afFy = m_stFTData.fFy;
    afFz = m_stFTData.fFz;
}

void CSensorFT::GetTorque(float& afTx, float& afTy, float& afTz) const
{
    afTx = m_stFTData.fTx;
    afTy = m_stFTData.fTy;
    afTz = m_stFTData.fTz;
}

void CSensorFT::GetRawForce(INT16& anFx, INT16& anFy, INT16& anFz) const
{
    anFx = m_stFTData.nRawFx;
    anFy = m_stFTData.nRawFy;
    anFz = m_stFTData.nRawFz;
}

void CSensorFT::GetRawTorque(INT16& anTx, INT16& anTy, INT16& anTz) const
{
    anTx = m_stFTData.nRawTx;
    anTy = m_stFTData.nRawTy;
    anTz = m_stFTData.nRawTz;
}

BOOL CSensorFT::IsForceOverload() const
{
    return m_bForceOverload;
}

BOOL CSensorFT::IsTorqueOverload() const
{
    return m_bTorqueOverload;
}

BOOL CSensorFT::CheckForceLimits(float afFx, float afFy, float afFz)
{
    float fMagnitude = sqrt(afFx*afFx + afFy*afFy + afFz*afFz);
    return CheckLimits(fMagnitude);
}

BOOL CSensorFT::CheckTorqueLimits(float afTx, float afTy, float afTz)
{
    float fMagnitude = sqrt(afTx*afTx + afTy*afTy + afTz*afTz);
    return CheckLimits(fMagnitude);
}

BOOL CSensorFT::SetCalibrationData(void* apData, UINT32 auSize)
{
    if (!apData || auSize != sizeof(ST_FT_CALIBRATION)) {
        DBG_LOG_ERROR("(%s) Invalid calibration data", "CSensorFT");
        return FALSE;
    }
    
    memcpy(&m_stFTCalibration, apData, sizeof(ST_FT_CALIBRATION));
    
    DBG_LOG_INFO("(%s) Calibration data loaded", "CSensorFT");
    
    return TRUE;
}

BOOL CSensorFT::LoadCalibrationFromFile(const TSTRING& astrFileName)
{
    std::ifstream file(astrFileName.c_str(), std::ios::binary);
    if (!file.is_open()) {
        DBG_LOG_ERROR("(%s) Cannot open calibration file: %s", "CSensorFT", astrFileName.c_str());
        return FALSE;
    }
    
    file.read(reinterpret_cast<char*>(&m_stFTCalibration), sizeof(ST_FT_CALIBRATION));
    file.close();
    
    if (!m_stFTCalibration.bIsValid) {
        DBG_LOG_ERROR("(%s) Invalid calibration data in file: %s", "CSensorFT", astrFileName.c_str());
        return FALSE;
    }
    
    DBG_LOG_INFO("(%s) Calibration loaded from file: %s", "CSensorFT", astrFileName.c_str());
    
    return TRUE;
}

BOOL CSensorFT::SaveCalibrationToFile(const TSTRING& astrFileName)
{
    if (!m_stFTCalibration.bIsValid) {
        DBG_LOG_ERROR("(%s) No valid calibration data to save", "CSensorFT");
        return FALSE;
    }
    
    std::ofstream file(astrFileName.c_str(), std::ios::binary);
    if (!file.is_open()) {
        DBG_LOG_ERROR("(%s) Cannot create calibration file: %s", "CSensorFT", astrFileName.c_str());
        return FALSE;
    }
    
    file.write(reinterpret_cast<const char*>(&m_stFTCalibration), sizeof(ST_FT_CALIBRATION));
    file.close();
    
    DBG_LOG_INFO("(%s) Calibration saved to file: %s", "CSensorFT", astrFileName.c_str());
    
    return TRUE;
}

void CSensorFT::ProcessRawData()
{
    // Convert raw data to engineering units
    m_stFTData.fFx = (float)m_stFTData.nRawFx / m_stFTCalibration.fForceDivider;
    m_stFTData.fFy = (float)m_stFTData.nRawFy / m_stFTCalibration.fForceDivider;
    m_stFTData.fFz = (float)m_stFTData.nRawFz / m_stFTCalibration.fForceDivider;
    m_stFTData.fTx = (float)m_stFTData.nRawTx / m_stFTCalibration.fTorqueDivider;
    m_stFTData.fTy = (float)m_stFTData.nRawTy / m_stFTCalibration.fTorqueDivider;
    m_stFTData.fTz = (float)m_stFTData.nRawTz / m_stFTCalibration.fTorqueDivider;
    
    // Apply calibration if valid
    if (m_stFTCalibration.bIsValid) {
        ApplyCalibration();
    }
    
    // Calculate magnitudes
    CalculateMagnitudes();
    
    // Check for overload conditions
    m_bForceOverload = !CheckForceLimits(m_stFTData.fFx, m_stFTData.fFy, m_stFTData.fFz);
    m_bTorqueOverload = !CheckTorqueLimits(m_stFTData.fTx, m_stFTData.fTy, m_stFTData.fTz);
    
    // Update timestamp
    m_stFTData.uTimestamp = read_timer();
    m_stFTData.bDataValid = TRUE;
}

void CSensorFT::CalculateMagnitudes()
{
    m_stFTData.fForcemagnitude = sqrt(m_stFTData.fFx*m_stFTData.fFx + 
                                     m_stFTData.fFy*m_stFTData.fFy + 
                                     m_stFTData.fFz*m_stFTData.fFz);
    
    m_stFTData.fTorqueMagnitude = sqrt(m_stFTData.fTx*m_stFTData.fTx + 
                                      m_stFTData.fTy*m_stFTData.fTy + 
                                      m_stFTData.fTz*m_stFTData.fTz);
}

void CSensorFT::ApplyCalibration()
{
    // Apply bias correction
    m_stFTData.fFx -= m_stFTCalibration.fBiasFx;
    m_stFTData.fFy -= m_stFTCalibration.fBiasFy;
    m_stFTData.fFz -= m_stFTCalibration.fBiasFz;
    m_stFTData.fTx -= m_stFTCalibration.fBiasTx;
    m_stFTData.fTy -= m_stFTCalibration.fBiasTy;
    m_stFTData.fTz -= m_stFTCalibration.fBiasTz;
    
    // Apply scale correction
    m_stFTData.fFx *= m_stFTCalibration.fScaleFx;
    m_stFTData.fFy *= m_stFTCalibration.fScaleFy;
    m_stFTData.fFz *= m_stFTCalibration.fScaleFz;
    m_stFTData.fTx *= m_stFTCalibration.fScaleTx;
    m_stFTData.fTy *= m_stFTCalibration.fScaleTy;
    m_stFTData.fTz *= m_stFTCalibration.fScaleTz;
}

void CSensorFT::UpdateRawData(INT16 anFx, INT16 anFy, INT16 anFz, INT16 anTx, INT16 anTy, INT16 anTz)
{
    m_stFTData.nRawFx = anFx;
    m_stFTData.nRawFy = anFy;
    m_stFTData.nRawFz = anFz;
    m_stFTData.nRawTx = anTx;
    m_stFTData.nRawTy = anTy;
    m_stFTData.nRawTz = anTz;
}
