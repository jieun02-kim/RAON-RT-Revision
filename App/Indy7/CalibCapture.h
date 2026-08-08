#pragma once
#include <string>
#include "FullDynControllerRT.h"

// Captures robot TCP poses for hand-eye calibration.
// [kv260-merge] ViSP-free reimplementation: the only math ViSP provided was
// rotation-matrix -> theta-u (axis-angle), which Eigen::AngleAxisd covers.
// Output stays byte-compatible with ViSP vpPoseVector::saveYAML so
// App/CalibUtils/eye_to_hand_calib.py keeps reading it unchanged.
class CalibCapture
{
public:
    explicit CalibCapture(const std::string& outDir = "calib_data");

    // Save current pose as <outDir>/pose_rPe_<N>.yaml
    // (6x1 pose vector: [tx ty tz, thetau_x thetau_y thetau_z])
    void Capture(const CControllerFullDynamicsRT::Pose& pose);

    void Reset();
    int  GetCount() const { return m_count; }

private:
    std::string m_outDir;
    int         m_count;
};
