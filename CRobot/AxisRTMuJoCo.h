/*****************************************************************************
*   Name: AxisRTMuJoCo.h
*   Author: Raimarius Delgado
*   Description: Hybrid RT control with MuJoCo visualization interface
*****************************************************************************/
#ifndef __AXIS__RT__MUJOCO__
#define __AXIS__RT__MUJOCO__


#include <atomic>
#include <memory>
#include <vector>
#include <string>
#include <mujoco/mujoco.h>
#include "AxisRT.h"
#include "MuJoCoVisualization.h" // Required for RTVisualizationState/VisualizationRTState

class CAxisRTMuJoCo : public CAxisRT {
public:
    CAxisRTMuJoCo(eAxisType aeAxisType, int mjJointId, double adEncRes = 65536,
                  double adGearRatio = 1.0, double adTransRatio = 1.0,
                  BOOL abAbsoluteEncoder = TRUE, BOOL abCCW = TRUE, BOOL abEnabled = TRUE);
    virtual ~CAxisRTMuJoCo();

    // RT-safe methods (called from RT thread)
    virtual void StepSimulation(double dt) override;
    void SetVisualizationEnabled(bool enabled) { m_bVisualizationEnabled = enabled; }

    // Visualization thread methods (called from non-RT thread)
    bool InitMuJoCo(mjModel* model, mjData* data);
    void UpdateMuJoCoVisualization();  // Non-RT: Update MuJoCo from RT state
    void StepMuJoCoPhysics();   // Non-RT: Step MuJoCo physics for external forces

    // State synchronization (lock-free)
    bool HasVisualizationUpdate() const { return m_rtToVizState.data_updated.load(); }
    void GetVisualizationState(double& pos, double& vel, double& torque, double& time);
    void SetExternalForcesFromMuJoCo(double torque, double fx, double fy, double fz);

    // MuJoCo accessors
    int GetMuJoCoJointId() const { return m_nMjJointId; }
    mjModel* GetMuJoCoModel() const { return m_pMjModel; }
    mjData* GetMuJoCoData() const { return m_pMjData; }

    // RT-safe state publishing (no locks)
    void PublishStateToVisualization();
protected:
    void UpdateExternalForcesFromVisualization();

private:
    // MuJoCo interface (only used in visualization thread)
    mjModel* m_pMjModel;
    mjData* m_pMjData;
    int m_nMjJointId;

    // Lock-free communication between RT and visualization threads
    RTVisualizationData m_rtToVizState;      // RT -> Visualization
    RTVisualizationData m_vizToRTState;      // Visualization -> RT

    // Visualization settings
    std::atomic<bool> m_bVisualizationEnabled;
    std::atomic<bool> m_bMuJoCoInitialized;

    // Synchronization state
    double m_dLastVizUpdateTime;
    double m_dVizUpdateRate;   // Hz for visualization updates
};

class CRobotRTMuJoCo {
public:
    CRobotRTMuJoCo();
    ~CRobotRTMuJoCo();

    // Inject config-driven axes (ownership is transferred)
    void SetAxes(std::vector<std::unique_ptr<CAxisRTMuJoCo>>&& axes);

    // Initialization
    bool InitRT();
    bool InitMuJoCo(mjModel* model, mjData* data);
    void Shutdown();

    // RT thread methods (call these from your 1kHz RT loop)
    void StepRTSimulation(double dt);
    void SetJointCommand(int jointId, double position);
    void SetJointVelocityCommand(int jointId, double velocity);
    void SetJointTorqueCommand(int jointId, double torque);

    // Non-RT thread methods (call these from visualization thread)
    void UpdateVisualization();  // ~30-60Hz
    void StepMuJoCoPhysics();   // Step MuJoCo for external forces

    // Communication between RT and visualization
    void UpdateMuJoCoFromRT();   // Called from proc_ethercat_control

    // State queries (RT-safe)
    double GetJointPosition(int jointId);
    double GetJointVelocity(int jointId);
    double GetJointTorque(int jointId);
    bool IsJointReady(int jointId);

    // Visualization control
    void SetVisualizationEnabled(bool enabled);
    bool IsVisualizationReady() const { return m_bMuJoCoReady.load(); }

    // Configuration from INDY7.cfg style
    bool ConfigureFromFile(const std::string& configFile);

private:
    std::vector<std::unique_ptr<CAxisRTMuJoCo>> m_axes;
    std::atomic<bool> m_bRTInitialized;
    std::atomic<bool> m_bMuJoCoReady;
    int m_nNumAxes = 0;
};

#endif // __AXIS__RT__MUJOCO__