/*****************************************************************************
*	Name: SensorFT.h
*	Author: Raimarius Delgado
*	Affiliation: RAIMLAB @ Myongji University
*	Description: Header for the CSensorFT class (Force/Torque Sensor)
*	Copyright: RAIMLAB (2025)
*****************************************************************************/
#ifndef __SENSOR_FT__
#define __SENSOR_FT__

#include "Sensor.h"

typedef struct stForceTorqueData
{
    // Raw values
    INT16 nRawFx, nRawFy, nRawFz;
    INT16 nRawTx, nRawTy, nRawTz;
    
    // Calibrated values
    float fFx, fFy, fFz;
    float fTx, fTy, fTz;
    
    // Magnitude values
    float fForcemagnitude;
    float fTorqueMagnitude;
    
    // Status
    BOOL bDataValid;
    UINT32 uTimestamp;

    stForceTorqueData()
    {
        nRawFx = nRawFy = nRawFz = 0;
        nRawTx = nRawTy = nRawTz = 0;
        fFx = fFy = fFz = 0.0f;
        fTx = fTy = fTz = 0.0f;
        fForcemagnitude = 0.0f;
        fTorqueMagnitude = 0.0f;
        bDataValid = FALSE;
        uTimestamp = 0;
    }
} ST_FT_DATA;

typedef struct stFTCalibration
{
    // Bias values
    float fBiasFx, fBiasFy, fBiasFz;
    float fBiasTx, fBiasTy, fBiasTz;
    
    // Scale factors
    float fScaleFx, fScaleFy, fScaleFz;
    float fScaleTx, fScaleTy, fScaleTz;
    
    // Conversion factors
    float fForceDivider;
    float fTorqueDivider;
    
    BOOL bIsValid;

    stFTCalibration()
    {
        fBiasFx = fBiasFy = fBiasFz = 0.0f;
        fBiasTx = fBiasTy = fBiasTz = 0.0f;
        fScaleFx = fScaleFy = fScaleFz = 1.0f;
        fScaleTx = fScaleTy = fScaleTz = 1.0f;
        fForceDivider = 50.0f;  // Default for NRMK
        fTorqueDivider = 2000.0f;  // Default for NRMK
        bIsValid = FALSE;
    }
} ST_FT_CALIBRATION;

class CSensorFT : public CSensor
{
public:
    CSensorFT(eSensorCommType aeComm, BOOL abEnabled = TRUE);
    virtual ~CSensorFT();

    // Override base class methods
    virtual BOOL Calibrate(eSensorCalibration aeCalType = eSensorCalibrationZero) override;
    virtual BOOL ReadData() override;
    virtual BOOL IsDataReady() override;

    // FT-specific methods
    virtual BOOL ZeroBias();
    virtual BOOL SetBias(float afFx, float afFy, float afFz, float afTx, float afTy, float afTz);
    virtual BOOL SetScale(float afFx, float afFy, float afFz, float afTx, float afTy, float afTz);
    virtual BOOL SetDividers(float afForceDivider, float afTorqueDivider);

    // Data access
    virtual ST_FT_DATA GetFTData() const { return m_stFTData; }
    virtual void GetForce(float& afFx, float& afFy, float& afFz) const;
    virtual void GetTorque(float& afTx, float& afTy, float& afTz) const;
    virtual void GetRawForce(INT16& anFx, INT16& anFy, INT16& anFz) const;
    virtual void GetRawTorque(INT16& anTx, INT16& anTy, INT16& anTz) const;
    
    // Individual getters
    virtual float GetFx() const { return m_stFTData.fFx; }
    virtual float GetFy() const { return m_stFTData.fFy; }
    virtual float GetFz() const { return m_stFTData.fFz; }
    virtual float GetTx() const { return m_stFTData.fTx; }
    virtual float GetTy() const { return m_stFTData.fTy; }
    virtual float GetTz() const { return m_stFTData.fTz; }
    
    virtual INT16 GetRawFx() const { return m_stFTData.nRawFx; }
    virtual INT16 GetRawFy() const { return m_stFTData.nRawFy; }
    virtual INT16 GetRawFz() const { return m_stFTData.nRawFz; }
    virtual INT16 GetRawTx() const { return m_stFTData.nRawTx; }
    virtual INT16 GetRawTy() const { return m_stFTData.nRawTy; }
    virtual INT16 GetRawTz() const { return m_stFTData.nRawTz; }

    // Magnitude calculations
    virtual float GetForceMagnitude() const { return m_stFTData.fForcemagnitude; }
    virtual float GetTorqueMagnitude() const { return m_stFTData.fTorqueMagnitude; }

    // Calibration management
    virtual ST_FT_CALIBRATION GetCalibration() const { return m_stFTCalibration; }
    virtual BOOL SetCalibrationData(void* apData, UINT32 auSize) override;
    virtual BOOL LoadCalibrationFromFile(const TSTRING& astrFileName) override;
    virtual BOOL SaveCalibrationToFile(const TSTRING& astrFileName) override;

    // Safety checks
    virtual BOOL IsForceOverload() const;
    virtual BOOL IsTorqueOverload() const;
    virtual BOOL CheckForceLimits(float afFx, float afFy, float afFz);
    virtual BOOL CheckTorqueLimits(float afTx, float afTy, float afTz);

protected:
    // Data processing
    virtual void ProcessRawData();
    virtual void CalculateMagnitudes();
    virtual void ApplyCalibration();
    virtual void UpdateRawData(INT16 anFx, INT16 anFy, INT16 anFz, INT16 anTx, INT16 anTy, INT16 anTz);

    // Member variables
    ST_FT_DATA m_stFTData;
    ST_FT_CALIBRATION m_stFTCalibration;
    
    // Overload detection
    BOOL m_bForceOverload;
    BOOL m_bTorqueOverload;
};

#endif //__SENSOR_FT__