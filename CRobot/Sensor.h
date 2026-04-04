/*****************************************************************************
*	Name: Sensor.h
*	Author: Raimarius Delgado
*	Affiliation: RAIMLAB @ Myongji University
*	Description: Header for the CSensor base class
*	Copyright: RAIMLAB (2025)
*****************************************************************************/
#ifndef __SENSOR__
#define __SENSOR__

#include "Defines.h"

typedef enum
{
    eSensorForceTorque = 0,
    eSensorVision,
    eSensorProximity,
    eSensorTemperature,
    eSensorPressure,
    eSensorAccelerometer,
    eSensorGyroscope,
    eSensorIMU,
    eSensorLidar,
    eSensorEncoder,
    eSensorTouch,
    eSensorUnknown = 0xFF,
} eSensorType;

typedef enum
{
    eSensorRS232 = 0,
    eSensorRS485,
    eSensorEthernet,
    eSensorEtherCAT,
    eSensorCAN,
    eSensorI2C,
    eSensorSPI,
    eSensorAnalog,
    eSensorUnknownCommType = 0x10,
} eSensorCommType;

typedef enum
{
    eSensorEmergency = 0x00,
    eSensorError,
    eSensorUnknownState,
    eSensorDeinit,
    eSensorDisconnected,
    eSensorInit,
    eSensorIdle,
    eSensorCalibrating,
    eSensorRunning,
    eSensorStopped,
    eSensorFault,
} eSensorState;

typedef enum
{
    eSensorCalibrationNone = 0,
    eSensorCalibrationZero,
    eSensorCalibrationBias,
    eSensorCalibrationScale,
    eSensorCalibrationFull,
} eSensorCalibration;

typedef struct stSensorInfo
{
    eSensorState eState;
    eSensorType eType;
    eSensorCommType eComm;
    eSensorCalibration eCalibration;
    UINT32 uSampleRate;
    UINT32 uFilterCutoff;
    BOOL bIsCalibrated;

    stSensorInfo()
    {
        eState = eSensorUnknownState;
        eType = eSensorUnknown;
        eComm = eSensorEtherCAT;
        eCalibration = eSensorCalibrationNone;
        uSampleRate = 1000;
        uFilterCutoff = 100;
        bIsCalibrated = FALSE;
    }
} ST_SENSOR_INFO;

typedef struct stSensorLimits
{
    BOOL bIsSet;
    double dMaxValue;
    double dMinValue;
    double dOverloadThreshold;
    double dWarningThreshold;

    stSensorLimits()
    {
        bIsSet = FALSE;
        dMaxValue = 0.0;
        dMinValue = 0.0;
        dOverloadThreshold = 0.0;
        dWarningThreshold = 0.0;
    }
} ST_SENSOR_LIMITS;

typedef struct stSensorStatus
{
    BOOL bOnline;
    BOOL bOverload;
    BOOL bWarning;
    BOOL bCalibrating;
    UINT8 uErrorCode;
    UINT32 uDataCount;
    UINT32 uLastUpdateTime;

    stSensorStatus()
    {
        bOnline = FALSE;
        bOverload = FALSE;
        bWarning = FALSE;
        bCalibrating = FALSE;
        uErrorCode = 0;
        uDataCount = 0;
        uLastUpdateTime = 0;
    }
} ST_SENSOR_STATUS;

class CSensor
{
public:
    CSensor(eSensorCommType aeComm, eSensorType aeType, BOOL abEnabled = TRUE);
    virtual ~CSensor();

protected:
    virtual BOOL InitHW();
    virtual BOOL InitSW();
    virtual BOOL DeInitHW();
    virtual BOOL DeInitSW();

public:
    // Basic sensor operations
    virtual BOOL Init() = 0;
    virtual BOOL DeInit() = 0;
    virtual BOOL Start() = 0;
    virtual BOOL Stop() = 0;
    virtual BOOL Reset() = 0;
    virtual BOOL Calibrate(eSensorCalibration aeCalType = eSensorCalibrationZero) = 0;
    virtual BOOL ReadData() = 0;
    virtual BOOL IsDataReady() = 0;

    // Configuration
    virtual BOOL SetSampleRate(UINT32 auRate) = 0;
    virtual BOOL SetFilterCutoff(UINT32 auCutoff) = 0;
    virtual BOOL SetLimits(double adMin, double adMax, double adOverload = 0.0, double adWarning = 0.0);
    virtual BOOL SetCalibrationData(void* apData, UINT32 auSize) = 0;
    virtual BOOL LoadCalibrationFromFile(const TSTRING& astrFileName) = 0;
    virtual BOOL SaveCalibrationToFile(const TSTRING& astrFileName) = 0;

    // Status and information
    virtual BOOL IsEnabled() const { return m_bEnabled; }
    virtual void SetEnabled(BOOL abEnabled) { m_bEnabled = abEnabled; }
    virtual BOOL IsOnline() const { return m_stSensorStatus.bOnline; }
    virtual BOOL IsCalibrated() const { return m_pstSensorInfo->bIsCalibrated; }
    virtual BOOL IsOverloaded() const { return m_stSensorStatus.bOverload; }
    virtual BOOL HasWarning() const { return m_stSensorStatus.bWarning; }
    virtual UINT8 GetErrorCode() const { return m_stSensorStatus.uErrorCode; }
    virtual UINT32 GetDataCount() const { return m_stSensorStatus.uDataCount; }
    virtual UINT32 GetSampleRate() const { return m_pstSensorInfo->uSampleRate; }
    virtual UINT32 GetFilterCutoff() const { return m_pstSensorInfo->uFilterCutoff; }

    // State management
    virtual void SetState(eSensorState aeState);
    virtual eSensorState GetState() const { return m_pstSensorInfo->eState; }
    virtual BOOL IsRunning() const { return m_pstSensorInfo->eState == eSensorRunning; }
    virtual BOOL IsStopped() const { return m_pstSensorInfo->eState == eSensorStopped; }
    virtual BOOL IsInError() const { return m_pstSensorInfo->eState == eSensorError || m_pstSensorInfo->eState == eSensorFault; }

    // Identification
    virtual void SetID(UINT16 ausID) { m_usSensorID = ausID; }
    virtual UINT16 GetID() const { return m_usSensorID; }
    virtual void SetName(const TSTRING& astrName) { m_strName = astrName; }
    virtual TSTRING GetName() const { return m_strName; }
    virtual void SetAliasPos(UINT16 ausAlias, UINT16 ausPos) { m_usSensorAlias = ausAlias; m_usSensorPos = ausPos; }

    // Type information
    virtual eSensorType GetSensorType() const { return m_pstSensorInfo->eType; }
    virtual void SetSensorType(eSensorType aeType) { m_pstSensorInfo->eType = aeType; }
    virtual eSensorCommType GetCommType() const { return m_pstSensorInfo->eComm; }
    virtual void SetCommType(eSensorCommType aeComm) { m_pstSensorInfo->eComm = aeComm; }

    // Limits and safety
    virtual ST_SENSOR_LIMITS GetSensorLimits() const { return *m_pstSensorLimits; }
    virtual ST_SENSOR_STATUS GetSensorStatus() const { return m_stSensorStatus; }
    virtual BOOL CheckLimits(double adValue);
    virtual BOOL IsValueInRange(double adValue);

    // EtherCAT specific (for derived classes)
    virtual void SetVendorInfo(UINT32 auVendorID, UINT32 auProductCode) { return; }
    virtual void SetDCInfo(BOOL abDCSupported, UINT16 ausActivateWord, INT32 anShiftTime) { return; }

    // Utility functions
    virtual void PrintSensorInfo() { return; }
    virtual TSTRING GetSensorTypeString() const;
    virtual TSTRING GetSensorStateString() const;
    virtual TSTRING GetSensorStateString(eSensorState aeState) const;  // Add this line

protected:
    // Helper functions
    virtual void UpdateStatus();
    virtual void CheckOverload();
    virtual void IncrementDataCount() { m_stSensorStatus.uDataCount++; }
    virtual void SetErrorCode(UINT8 auErrorCode) { m_stSensorStatus.uErrorCode = auErrorCode; }
    virtual void ClearError() { m_stSensorStatus.uErrorCode = 0; SetState(eSensorIdle); }

    // Member variables
    UINT16 m_usSensorID;
    UINT16 m_usSensorAlias;
    UINT16 m_usSensorPos;
    BOOL m_bEnabled;
    TSTRING m_strName;

    ST_SENSOR_INFO* m_pstSensorInfo;
    ST_SENSOR_LIMITS* m_pstSensorLimits;
    ST_SENSOR_STATUS m_stSensorStatus;

private:
    // Private helper functions
    BOOL ValidateRange(double adMin, double adMax);
};

#endif //__SENSOR__
