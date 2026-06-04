#pragma once
#include <string>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpPoseVector.h>
#include "FullDynControllerRT.h"

// Captures robot TCP poses for hand-eye calibration.
// All ViSP dependencies are confined to this class.
class CalibCapture
{
public:
    explicit CalibCapture(const std::string& outDir = "calib_data");

    // Convert current pose to vpPoseVector and save as YAML.
    // File: <outDir>/pose_rPe_<N>.yaml  (N = 0-based index)
    void Capture(const CControllerFullDynamicsRT::Pose& pose);

    void Reset();
    int  GetCount() const { return m_count; }

private:
    std::string m_outDir;
    int         m_count;

    static vpHomogeneousMatrix ToMatrix(const CControllerFullDynamicsRT::Pose& pose);
};
