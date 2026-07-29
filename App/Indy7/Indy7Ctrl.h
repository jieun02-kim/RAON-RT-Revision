/*****************************************************************************
*	Name: RobotUSurgery.h
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Header for the CRobotIndy7 class
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
#ifndef __ROBOT_EXAMPLE_APP__
#define __ROBOT_EXAMPLE_APP__

#include <list>
#include <queue>
#include <atomic>
#include <ctime>

#include "Robot.h"
#include "AxisNRMKCore.h"
#include "SensorNRMKEndTool.h"
#include "FullDynControllerRT.h"
#include "DataRecorder.h"
#include "CalibCapture.h"
// [kv260-merge] VisualServo (ViSP AprilTag) removed from the control app:
// it opens the RealSense directly and cannot coexist with the perception
// pipeline. Goal poses come from ROS2 (/pick_target_base) instead.
// VisualServo.{h,cpp} remain in-tree (unbuilt) as closed-loop reference.
#include "ROS2PickBridge.h"


typedef std::vector<double> VECDOUBLE;
typedef std::list<UINT64>   LISTULONG;
typedef std::list<INT32>	LISTINT;


typedef struct stDataLog
{
	LISTULONG	vecTimestamp;
	LISTINT		vecTarPos;
	LISTINT		vecActPos;
	LISTINT		vecActTor;

	stDataLog()
	{
		vecTimestamp.clear();
		vecTarPos.clear();
		vecActPos.clear();
		vecActTor.clear();
	}

}ST_DATALOG;

class CRobotIndy7 : public CRobot
{
public:
	CRobotIndy7(CConfigRobot* apcConfig = NULL);
	virtual ~CRobotIndy7();

public:
	virtual BOOL	Init					(BOOL abSim);
	virtual BOOL	DeInit					(	);
	void	WriteDataLog					(	);

	enum eISOHWState { eISO_HW_IDLE=0, eISO_HW_TO_TARGET, eISO_HW_TO_CLEARANCE, eISO_HW_DONE };
	enum eRectState  { eRECT_IDLE=0, eRECT_TO_CORNER };
	enum eApproachState { eAPPROACH_IDLE=0, eAPPROACH_MOVING, eAPPROACH_STAGING };
	
	/* Controller */
	CControllerFullDynamicsRT* GetController() { return m_pController; }
    BOOL InitController(const TSTRING& astrURDFPath);
    BOOL EnableController(BOOL abEnable);
    BOOL SetControllerMode(CControllerFullDynamicsRT::eControlMode aeMode);
    BOOL SetControllerGains(const std::vector<double>& avKp, const std::vector<double>& avKd);
	BOOL IsMoving();

private:
	TSTRING		m_strDataLog;
	ST_DATALOG	m_stDataLog[32];

protected:
	virtual BOOL	InitEtherCAT			(	);
	virtual BOOL	InitConfig				(	);
	virtual void	DoAgingTest				(	);

	CAxisNRMKCore**			m_pEcatAxis;
	CSensorNRMKEndTool**	m_pEcatSensor;

	/* TEMPORARY */
	char	m_cKeyPress;
	void	DoInput						(	);
	void	DoHoming() {};

protected:
	friend void	proc_main_control(void*);
	friend void proc_ethercat_control(void*);
	friend void	proc_keyboard_control(void*);
	friend void proc_terminal_output(void*);
	friend void proc_logger(void*);
	friend FILE* make_csv(CRobotIndy7*);

private:
	BOOL m_bEcatOP;
	UINT32	m_nEcatCycle;
	BOOL m_bEnableTriangleControl{false};

	/* Controller */
	CControllerFullDynamicsRT*	m_pController;
	BOOL m_bRTControllerEnabled;

	// control vectors 
    std::vector<double> m_vCurrentPos;
    std::vector<double> m_vCurrentVel;
    std::vector<double> m_vCurrentTor;
    std::vector<double> m_vOutputTorque;

	//=====================================================
	//jieun

	// TCP Trajectory Logging
	LogRingBuffer            m_logBuffer;
	std::atomic<bool>        m_bLogTrigger{false};

	// ISO Hardware Test
	std::atomic<bool>               m_bIsoHWTrigger{false};
	eISOHWState                     m_eISOHWState{eISO_HW_IDLE};
	int                             m_nISOPointIdx{0};
	int                             m_nISOCycle{0};
	bool                            m_bISOCmdSent{false};
	double                          m_isoHWRecords[5][10][3];
	CControllerFullDynamicsRT::Pose m_isoTargets[5];
	CControllerFullDynamicsRT::Pose m_isoClearance;

	// Rectangle (Square) Motion — 'n' key
	std::atomic<bool>               m_bRectTrigger{false};
	eRectState                      m_eRectState{eRECT_IDLE};
	int                             m_nRectCornerIdx{0};
	int                             m_nRectWaitCount{0};
	CControllerFullDynamicsRT::Pose m_rectCorners[4];
	RigidBodyDynamics::Math::VectorNd m_rectJointTargets[4];

	CControllerFullDynamicsRT::Pose	m_Pose;
	void SaveISOHWResults();
	void SaveRobotPose();
	int  m_nPoseCapture{0};

	// [kv260-merge] Gate 2b — perception pipeline bridge ('p'/digit/'v').
	// Bridge threads are non-RT; the RT loop only uses its wait-free API.
	// v1 approach = position-only IK (keeps current TCP orientation) via the
	// proven 'n'-key sequence; top-down fixed R is deferred to Phase 5.
	CROS2PickBridge*       m_pPickBridge{nullptr};
	eApproachState         m_eApproachState{eAPPROACH_IDLE};
	static constexpr double APPROACH_DURATION_S = 3.0;

	// [kv260-merge] Ready-seed approach (2026-07-27): every goal is solved
	// from the init-computed q_ready (deterministic — same target, same
	// solution, regardless of where the operator parked the arm). If the
	// solution is within IK_DQ_MAX_RAD of the current posture we go direct;
	// otherwise we STAGE: joint-move to q_ready (the 'b'-class proven
	// primitive), settle, then fire the goal leg. Leg 2 only fires if mode
	// and servo are still exactly as we left them — anything else cancels
	// (a deferred surprise motion is the j→v trap all over again).
	BOOL TryReadyApproach(const CROS2PickBridge::Goal& astGoal, BOOL abAllowStage);
	CROS2PickBridge::Goal  m_stStagedGoal{};
	int                    m_nStageSettleCnt{0};

	// [kv260-merge] Approach accuracy record (2026-07-29). The goal in flight
	// (class + vision std travel with it, so the plot can tell a perception
	// error from a control error) and the FK TCP measured after the FIRST
	// trajectory — i.e. BEFORE refinement — so the report shows what the CTC
	// droop was and how much refine actually recovered.
	CROS2PickBridge::Goal  m_stActiveGoal{};
	RigidBodyDynamics::Math::Vector3d m_vFirstTcp;
	bool                   m_bFirstTcpSet{false};
	void EmitApproachReport();
	static constexpr double STAGE_VEL_EPS    = 0.05;  // [rad/s] Σ|qd| gate
	static constexpr int    STAGE_SETTLE_CYC = 200;   // 0.2 s below eps

	// [kv260-merge] HOME: the 'p'-menu entry snapshots the CURRENT JOINTS
	// (no IK — returning to a recorded q has no branch-jump risk by
	// construction); 'b' replays them with the E13 speed-scaled quintic.
	RigidBodyDynamics::Math::VectorNd m_vHomeQ;
	bool m_bHomeSet{false};

	// [kv260-merge] Iterative position refinement — closes the CTC
	// steady-state error (gate 2 measured ~16 cm: no integral action +
	// harmonic-drive friction). After a trajectory settles, the FK error
	// vs the desired position is ADDED as a bias to the commanded target
	// and IK re-runs; the droop is locally near-constant so this converges
	// in a few passes. Runs after 'a' and after every vision approach.
	// Takes the full pose: with the E13 orientation-constrained IK the refine
	// re-targets must carry the SAME rotation as the original command — the
	// old Vector3d overload left m_stRefineCmd.m_rotation at identity.
	void StartRefine(const CControllerFullDynamicsRT::Pose& astDesired);
	bool m_bRefineActive{false};
	int  m_nRefineIter{0};
	int  m_nRefineSettleCnt{0};
	RigidBodyDynamics::Math::Vector3d m_vRefineDesired;
	CControllerFullDynamicsRT::Pose   m_stRefineCmd;
	// Tuning (2026-07-26 CSV analysis): the plant is stiction-dominated —
	// the arm STOPS inside a friction dead-band whose offset varies per
	// move, so full-gain bias oscillates around the target. Damped bias +
	// velocity-gated measurement converges instead.
	static constexpr double REFINE_TOL_M      = 0.012;  // stiction floor ~10-15 mm at stock gains
	static constexpr int    REFINE_MAX_ITER   = 6;
	static constexpr double REFINE_DAMPING    = 0.65;   // bias += a*err
	static constexpr double REFINE_TRAJ_T_S   = 1.5;
	static constexpr double REFINE_VEL_EPS    = 0.02;   // [rad/s] |qd| gate
	static constexpr int    REFINE_SETTLE_CYC = 300;    // 0.3 s below eps

	//=====================================================

public:
	//=====================================================



};

//=====================================================
// jieun — ViSP / Calibration
// Defined in Indy7Ctrl.cpp; visible to any TU that includes this header.
extern CalibCapture s_calibCapture;
//=====================================================

#endif //__ROBOT_EXAMPLE_APP__
