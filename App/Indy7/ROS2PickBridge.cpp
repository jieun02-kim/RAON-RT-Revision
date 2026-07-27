/*****************************************************************************
*	Name: ROS2PickBridge.cpp
*	[kv260-merge] Gate 2b — ROS2 bridge to the KV260 perception pipeline.
*	See ROS2PickBridge.h for the threading contract.
*****************************************************************************/
#include "ROS2PickBridge.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

using namespace std::chrono_literals;

static const char* TOPIC_TARGET     = "/pick_target_base";
static const char* TOPIC_DETECTIONS = "/detections";
static const char* PICK_LOGIC_NODE  = "/pick_logic_node";
static const char* PARAM_DESIRED    = "desired_class";
static const char* CLASS_PERSON     = "person";

constexpr double CROS2PickBridge::DEF_BOX[6];

CROS2PickBridge::CROS2PickBridge()
{
    for (int i = 0; i < 6; i++)
        m_dBox[i] = DEF_BOX[i];
    std::memset(&m_stGoal, 0, sizeof(m_stGoal));
}

CROS2PickBridge::~CROS2PickBridge()
{
    DeInit();
}

void
CROS2PickBridge::SetWorkspaceBox(double adXMin, double adXMax, double adYMin,
                                 double adYMax, double adZMin, double adZMax)
{
    m_dBox[0] = adXMin; m_dBox[1] = adXMax;
    m_dBox[2] = adYMin; m_dBox[3] = adYMax;
    m_dBox[4] = adZMin; m_dBox[5] = adZMax;
}

void
CROS2PickBridge::SetGate(int anSamples, double adStdGateM, double adTimeoutS)
{
    m_nGateSamples = anSamples;
    m_dStdGateM    = adStdGateM;
    m_dTimeoutS    = adTimeoutS;
}

BOOL
CROS2PickBridge::Init()
{
    if (m_bInit)
        return TRUE;

    try
    {
        if (!rclcpp::ok())
        {
            // The app owns SIGINT/SIGTERM (main.cpp) — rclcpp must NOT
            // install its own handlers or Ctrl+C would bypass CRobot::DeInit.
            rclcpp::init(0, nullptr, rclcpp::InitOptions(),
                         rclcpp::SignalHandlerOptions::None);
            m_bOwnContext = TRUE;
        }

        m_pNode = std::make_shared<rclcpp::Node>("raon_pick_bridge");

        m_pSubTarget = m_pNode->create_subscription<my_interfaces::msg::PickTarget3D>(
            TOPIC_TARGET, rclcpp::QoS(10),
            [this](const my_interfaces::msg::PickTarget3D::SharedPtr msg)
            { OnTarget(msg); });

        m_pSubDetections = m_pNode->create_subscription<my_interfaces::msg::DetectionArray>(
            TOPIC_DETECTIONS, rclcpp::QoS(10),
            [this](const my_interfaces::msg::DetectionArray::SharedPtr msg)
            { OnDetections(msg); });

        m_pParamClient = std::make_shared<rclcpp::AsyncParametersClient>(
            m_pNode, PICK_LOGIC_NODE);

        m_bStop = false;
        m_thSpin   = std::thread(&CROS2PickBridge::SpinLoop, this);
        m_thWorker = std::thread(&CROS2PickBridge::WorkerLoop, this);
    }
    catch (const std::exception& e)
    {
        printf("[PickBridge] Init failed: %s\n", e.what());
        m_pParamClient.reset();
        m_pSubDetections.reset();
        m_pSubTarget.reset();
        m_pNode.reset();
        return FALSE;
    }

    m_bInit = TRUE;
    printf("[PickBridge] up — sub %s, %s | param target %s\n",
           TOPIC_TARGET, TOPIC_DETECTIONS, PICK_LOGIC_NODE);
    printf("[PickBridge] keys: 'p'=object menu, digit=select, 'v'=approach, "
           "'b'=go home (N=%d, std<%.0f mm, z+%.2f m)\n",
           m_nGateSamples, m_dStdGateM * 1e3, m_dZMarginM);
    return TRUE;
}

BOOL
CROS2PickBridge::DeInit()
{
    if (!m_bInit)
        return TRUE;
    m_bInit = FALSE;

    // Leave the pipeline neutral: desired_class is a LIVE param inside
    // pick_logic and SURVIVES this app — a stale class kept /pick_target_base
    // hot across restarts (2026-07-27). One quick best-effort attempt only;
    // teardown must stay snappy (E7).
    try
    {
        if (m_pParamClient && m_pParamClient->service_is_ready())
        {
            auto cFut = m_pParamClient->set_parameters(
                {rclcpp::Parameter(PARAM_DESIRED, std::string())});
            if (cFut.wait_for(500ms) == std::future_status::ready)
                printf("[PickBridge] desired_class cleared (session end)\n");
        }
    }
    catch (...) { /* teardown best-effort */ }

    m_bStop = true;
    if (m_bOwnContext && rclcpp::ok())
        rclcpp::shutdown();          // makes spin() return
    if (m_thSpin.joinable())
        m_thSpin.join();
    if (m_thWorker.joinable())
        m_thWorker.join();

    m_pParamClient.reset();
    m_pSubDetections.reset();
    m_pSubTarget.reset();
    m_pNode.reset();
    printf("[PickBridge] down\n");
    return TRUE;
}

/* ---------------------------------------------------------------- [RT] -- */

BOOL
CROS2PickBridge::FeedDigit(char acDigit)
{
    if (acDigit < '0' || acDigit > '9')
        return FALSE;
    if (!m_bMenuOpen.load(std::memory_order_acquire))
        return FALSE;
    m_nDigit.store(acDigit - '0', std::memory_order_release);
    return TRUE;
}

BOOL
CROS2PickBridge::PopGoal(Goal& astOut)
{
    if (!m_bGoalReady.load(std::memory_order_acquire))
        return FALSE;
    astOut = m_stGoal;   // producer never writes while ready==true
    m_bGoalReady.store(false, std::memory_order_release);
    return TRUE;
}

/* ------------------------------------------------------------- threads -- */

void
CROS2PickBridge::SpinLoop()
{
    rclcpp::executors::SingleThreadedExecutor cExec;
    cExec.add_node(m_pNode);
    cExec.spin();    // returns on rclcpp::shutdown()
}

void
CROS2PickBridge::WorkerLoop()
{
    // Fresh session: neutralize any desired_class a previous app run left in
    // pick_logic (LIVE param — it outlives us). Without this the pipeline
    // keeps publishing the stale class from the moment we boot.
    if (SetDesiredClass(std::string()))
        printf("[PickBridge] desired_class reset for a fresh session\n");

    while (!m_bStop.load() && rclcpp::ok())
    {
        std::this_thread::sleep_for(50ms);

        if (m_bMenuReq.exchange(false))
            ShowMenu();

        const int nSel = m_nDigit.exchange(-1);
        if (nSel >= 0)
            HandleDigit(nSel);

        if (m_bApproachReq.exchange(false))
            StartCollect();

        TickCollect();
        TickLockWatch();
        TickSeedPersist();
    }
}

/* ------------------------------------------------- ready-seed persistence -- */

const char*
CROS2PickBridge::ReadySeedPath()
{
    static char s_acPath[512] = {0};
    if (s_acPath[0] == '\0')
    {
        const char* pcHome = getenv("HOME");
        snprintf(s_acPath, sizeof(s_acPath), "%s/.indy7_ready_seed",
                 (pcHome != NULL) ? pcHome : "/tmp");
    }
    return s_acPath;
}

void
CROS2PickBridge::PersistReadySeed(const double* aqRad, int anDof)
{
    if (aqRad == NULL || anDof <= 0) return;
    if (anDof > 8) anDof = 8;
    for (int i = 0; i < anDof; i++) m_adSeedMail[i] = aqRad[i];
    m_nSeedMailN.store(anDof, std::memory_order_release);
}

void
CROS2PickBridge::TickSeedPersist()
{
    const int nN = m_nSeedMailN.load(std::memory_order_acquire);
    if (nN <= 0) return;
    double adQ[8];
    for (int i = 0; i < nN; i++) adQ[i] = m_adSeedMail[i];
    m_nSeedMailN.store(0, std::memory_order_release);

    FILE* pFile = fopen(ReadySeedPath(), "w");
    if (pFile == NULL)
    {
        printf("[PickBridge] WARN: cannot write %s\n", ReadySeedPath());
        return;
    }
    for (int i = 0; i < nN; i++) fprintf(pFile, "%.9f ", adQ[i]);
    fprintf(pFile, "\n");
    fclose(pFile);
    printf("[PickBridge] operator seed persisted → %s (next boot anchors here)\n",
           ReadySeedPath());
}

/* ----------------------------------------------------------- callbacks -- */

void
CROS2PickBridge::OnTarget(const my_interfaces::msg::PickTarget3D::SharedPtr apMsg)
{
    std::lock_guard<std::mutex> cLock(m_Mtx);
    m_stLatest.dX       = apMsg->x;
    m_stLatest.dY       = apMsg->y;
    m_stLatest.dZ       = apMsg->z;
    m_stLatest.strClass = apMsg->class_name;
    m_stLatest.bValid   = apMsg->target_valid;
    m_stLatest.bDepth   = apMsg->depth_valid;
    m_stLatest.tStamp   = std::chrono::steady_clock::now();
    m_bHaveSample = true;

    if (m_bCollecting &&
        apMsg->target_valid && apMsg->depth_valid &&
        m_stLatest.strClass == m_strCollectClass &&
        (int)m_vCollect.size() < m_nGateSamples)
    {
        m_vCollect.push_back(m_stLatest);
    }
}

void
CROS2PickBridge::OnDetections(const my_interfaces::msg::DetectionArray::SharedPtr apMsg)
{
    const SteadyTP tNow = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> cLock(m_Mtx);
    for (const auto& det : apMsg->detections)
    {
        auto& rEntry  = m_mapSeen[det.class_name];
        rEntry.first  += 1;
        rEntry.second  = tNow;
    }
}

/* ---------------------------------------------------------------- menu -- */

void
CROS2PickBridge::ShowMenu()
{
    const SteadyTP tNow = std::chrono::steady_clock::now();
    bool bPersonNear = false;

    std::vector<std::pair<std::string, int>> vVisible;
    {
        std::lock_guard<std::mutex> cLock(m_Mtx);
        m_bAnnounceLock = false;   // new menu supersedes the previous session
        for (auto it = m_mapSeen.begin(); it != m_mapSeen.end();)
        {
            const double dAge =
                std::chrono::duration<double>(tNow - it->second.second).count();
            if (dAge > MENU_WINDOW_S)
            {
                it = m_mapSeen.erase(it);
                continue;
            }
            if (it->first == CLASS_PERSON)
                bPersonNear = true;
            else if (it->second.first >= MENU_MIN_COUNT)
                vVisible.push_back({it->first, it->second.first});
            it->second.first = 0;   // window restarts after every menu
            ++it;
        }
    }
    std::sort(vVisible.begin(), vVisible.end());   // deterministic numbering

    printf("\n[PickBridge] ---- visible objects (last %.1f s) ----\n", MENU_WINDOW_S);
    m_vMenu.clear();
    if (vVisible.empty())
        printf("[PickBridge]   (none — check pipeline / put an object in view)\n");
    int nIdx = 1;
    for (const auto& rV : vVisible)
    {
        printf("[PickBridge]   %d) %-16s (%d frames)\n", nIdx, rV.first.c_str(), rV.second);
        m_vMenu.push_back(rV.first);
        nIdx++;
    }
    printf("[PickBridge]   %d) AUTO (clear desired_class — best object)\n", nIdx);
    m_vMenu.push_back("");   // sentinel: auto
    printf("[PickBridge]   %d) record HOME here (return later with 'b')\n", nIdx + 1);
    if (bPersonNear)
        printf("[PickBridge]   ! person in view — pick_logic will veto if close\n");
    printf("[PickBridge] select 1-%d (0 = cancel)\n", nIdx + 1);
    m_bMenuOpen.store(true, std::memory_order_release);
}

void
CROS2PickBridge::HandleDigit(int anSel)
{
    if (!m_bMenuOpen.load(std::memory_order_acquire))
        return;
    m_bMenuOpen.store(false, std::memory_order_release);

    if (anSel == 0)
    {
        printf("[PickBridge] menu cancelled\n");
        return;
    }
    if (anSel == (int)m_vMenu.size() + 1)   // "record HOME" — last menu entry
    {
        m_bHomeRecordReq.store(true, std::memory_order_release);
        printf("[PickBridge] HOME record requested — RT snapshots the current "
               "joints ('b' returns there)\n");
        return;
    }
    if (anSel > (int)m_vMenu.size())
    {
        printf("[PickBridge] no entry %d — menu closed\n", anSel);
        return;
    }

    const std::string strClass = m_vMenu[anSel - 1];
    if (SetDesiredClass(strClass))
    {
        {
            std::lock_guard<std::mutex> cLock(m_Mtx);
            m_strSelected   = strClass;
            m_bAnnounceLock = true;    // session starts: narrate lock state
            m_bShownLocked  = false;
            m_bLockedPrev   = false;
        }
        if (strClass.empty())
            printf("[PickBridge] desired_class cleared → AUTO select\n");
        else
            printf("[PickBridge] desired_class='%s' set — waiting for lock\n",
                   strClass.c_str());
    }
}

BOOL
CROS2PickBridge::SetDesiredClass(const std::string& astrClass)
{
    for (int nTry = 1; nTry <= 3; nTry++)
    {
        try
        {
            if (!m_pParamClient->service_is_ready() &&
                !m_pParamClient->wait_for_service(2s))
            {
                printf("[PickBridge] %s param service not up (try %d/3)\n",
                       PICK_LOGIC_NODE, nTry);
                continue;
            }
            auto cFuture = m_pParamClient->set_parameters(
                {rclcpp::Parameter(PARAM_DESIRED, astrClass)});
            if (cFuture.wait_for(2s) != std::future_status::ready)
            {
                printf("[PickBridge] param set timed out (try %d/3)\n", nTry);
                continue;
            }
            const auto vResults = cFuture.get();
            if (!vResults.empty() && vResults[0].successful)
                return TRUE;
            printf("[PickBridge] param set rejected: %s\n",
                   vResults.empty() ? "empty result"
                                    : vResults[0].reason.c_str());
            return FALSE;
        }
        catch (const std::exception& e)
        {
            printf("[PickBridge] param set error: %s (try %d/3)\n", e.what(), nTry);
        }
    }
    printf("[PickBridge] FAILED to set desired_class — is the pipeline running?\n");
    return FALSE;
}

/* ------------------------------------------------------ statistics gate -- */

void
CROS2PickBridge::StartCollect()
{
    if (m_bCollecting)
    {
        printf("[PickBridge] already collecting — 'v' ignored\n");
        return;
    }
    if (m_bGoalReady.load(std::memory_order_acquire))
    {
        printf("[PickBridge] previous goal not consumed yet (RT side OP/enabled?)\n");
        return;
    }

    std::string strClass;
    {
        std::lock_guard<std::mutex> cLock(m_Mtx);
        const double dAge = m_bHaveSample
            ? std::chrono::duration<double>(
                  std::chrono::steady_clock::now() - m_stLatest.tStamp).count()
            : 1e9;
        if (!m_bHaveSample || dAge > LOCK_MAX_AGE_S || !m_stLatest.bValid)
        {
            printf("[PickBridge] no live valid target — select ('p') and wait "
                   "for TARGET LOCKED before 'v'\n");
            return;
        }
        strClass = m_stLatest.strClass;
        m_vCollect.clear();
        m_strCollectClass = strClass;
        m_bCollecting     = true;
        m_tCollectDeadline = std::chrono::steady_clock::now() +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(m_dTimeoutS));
    }
    printf("[PickBridge] collecting %d frames of '%s'...\n",
           m_nGateSamples, strClass.c_str());
}

void
CROS2PickBridge::TickCollect()
{
    std::vector<Sample> vDone;
    std::string strClass;
    {
        std::lock_guard<std::mutex> cLock(m_Mtx);
        if (!m_bCollecting)
            return;
        const bool bFull    = (int)m_vCollect.size() >= m_nGateSamples;
        const bool bTimeout = std::chrono::steady_clock::now() > m_tCollectDeadline;
        if (!bFull && !bTimeout)
            return;
        m_bCollecting = false;
        if (!bFull)
        {
            printf("[PickBridge] GATE FAIL: timeout — %zu/%d samples of '%s' "
                   "(unstable/occluded?)\n",
                   m_vCollect.size(), m_nGateSamples, m_strCollectClass.c_str());
            return;
        }
        vDone    = m_vCollect;
        strClass = m_strCollectClass;
    }

    double dMean[3] = {0, 0, 0}, dStd[3] = {0, 0, 0};
    for (const auto& rS : vDone)
    {
        dMean[0] += rS.dX; dMean[1] += rS.dY; dMean[2] += rS.dZ;
    }
    for (int i = 0; i < 3; i++)
        dMean[i] /= vDone.size();
    for (const auto& rS : vDone)
    {
        const double d[3] = {rS.dX - dMean[0], rS.dY - dMean[1], rS.dZ - dMean[2]};
        for (int i = 0; i < 3; i++)
            dStd[i] += d[i] * d[i];
    }
    for (int i = 0; i < 3; i++)
        dStd[i] = std::sqrt(dStd[i] / vDone.size());

    if (dStd[0] > m_dStdGateM || dStd[1] > m_dStdGateM || dStd[2] > m_dStdGateM)
    {
        printf("[PickBridge] GATE FAIL: std mm (%.1f, %.1f, %.1f) > %.1f — "
               "object/camera moving?\n",
               dStd[0] * 1e3, dStd[1] * 1e3, dStd[2] * 1e3, m_dStdGateM * 1e3);
        return;
    }

    const double dGoal[3] = {dMean[0], dMean[1], dMean[2] + m_dZMarginM};
    if (dGoal[0] < m_dBox[0] || dGoal[0] > m_dBox[1] ||
        dGoal[1] < m_dBox[2] || dGoal[1] > m_dBox[3] ||
        dGoal[2] < m_dBox[4] || dGoal[2] > m_dBox[5])
    {
        printf("[PickBridge] GATE FAIL: goal (%.3f, %.3f, %.3f) outside workspace "
               "box x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]\n",
               dGoal[0], dGoal[1], dGoal[2],
               m_dBox[0], m_dBox[1], m_dBox[2], m_dBox[3], m_dBox[4], m_dBox[5]);
        return;
    }
    const double dRxy = std::sqrt(dGoal[0] * dGoal[0] + dGoal[1] * dGoal[1]);
    if (dRxy > DEF_RMAX_XY)
    {
        printf("[PickBridge] GATE FAIL: goal r_xy %.3f m > reach %.2f m — "
               "move the object closer to the robot\n", dRxy, DEF_RMAX_XY);
        return;
    }

    // SPSC handshake: ready==false here (checked in StartCollect), safe to write.
    m_stGoal.dX = dGoal[0];
    m_stGoal.dY = dGoal[1];
    m_stGoal.dZ = dGoal[2];
    std::snprintf(m_stGoal.szClass, sizeof(m_stGoal.szClass), "%s", strClass.c_str());
    for (int i = 0; i < 3; i++)
        m_stGoal.dStdMM[i] = dStd[i] * 1e3;
    m_stGoal.nSamples = (int)vDone.size();
    m_bGoalReady.store(true, std::memory_order_release);

    printf("[PickBridge] GOAL READY: %s → (%.3f, %.3f, %.3f) "
           "[std mm %.1f/%.1f/%.1f, n=%d] — RT will start the approach\n",
           strClass.c_str(), dGoal[0], dGoal[1], dGoal[2],
           dStd[0] * 1e3, dStd[1] * 1e3, dStd[2] * 1e3, (int)vDone.size());

    {   // session handed to RT — stop narrating lock flicker
        std::lock_guard<std::mutex> cLock(m_Mtx);
        m_bAnnounceLock = false;
    }
}

/* ----------------------------------------------------------- lock watch -- */

void
CROS2PickBridge::TickLockWatch()
{
    bool bLocked = false;
    Sample stCur;
    int nPrint = 0;   // 0=silent, 1=locked, 2=lost
    {
        std::lock_guard<std::mutex> cLock(m_Mtx);
        const auto tNow = std::chrono::steady_clock::now();
        if (m_bHaveSample)
        {
            const double dAge = std::chrono::duration<double>(
                tNow - m_stLatest.tStamp).count();
            bLocked = (dAge < LOCK_MAX_AGE_S) && m_stLatest.bValid &&
                      (m_strSelected.empty() ||
                       m_stLatest.strClass == m_strSelected);
            stCur = m_stLatest;
        }
        // RT-visible state tracks the instantaneous lock unconditionally.
        m_bLocked.store(bLocked, std::memory_order_release);

        if (m_bAnnounceLock)
        {
            if (bLocked)
            {
                if (!m_bShownLocked) { nPrint = 1; m_bShownLocked = true; }
            }
            else
            {
                if (m_bLockedPrev)
                    m_tLostSince = tNow;   // falling edge — start debounce
                if (m_bShownLocked &&
                    std::chrono::duration<double>(tNow - m_tLostSince).count()
                        >= LOCK_LOST_DEBOUNCE_S)
                {
                    nPrint = 2;
                    m_bShownLocked = false;
                }
            }
        }
        m_bLockedPrev = bLocked;
    }

    if (nPrint == 1)
        printf("[PickBridge] TARGET LOCKED: %s @ base(%.3f, %.3f, %.3f) — "
               "'v' to approach\n",
               stCur.strClass.c_str(), stCur.dX, stCur.dY, stCur.dZ);
    else if (nPrint == 2)
        printf("[PickBridge] target lost (>%.0f s) — reposition object or "
               "'p' for a new menu\n", LOCK_LOST_DEBOUNCE_S);
}
