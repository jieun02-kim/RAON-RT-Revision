#include "VisualServo.h"

// ViSP/RealSense 헤더는 이 .cpp에만 포함 — 헤더 충돌 방지
#include <visp3/core/vpImage.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpPoseVector.h>
#include <visp3/core/vpArray2D.h>
#include <visp3/core/vpTranslationVector.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/core/vpRxyzVector.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <librealsense2/rs.hpp>

#include <cstdio>
#include <cmath>

// T_offset: tag x축 = robot -x 이므로 -0.15 → robot +x 방향으로 15cm 오프셋
// TCP z = 로봇 -x (마커 방향), TCP y = 로봇 +z (천장 방향)
static const vpHomogeneousMatrix s_Toffset(
    vpTranslationVector(-0.15, 0.0, 0.0),
    vpRotationMatrix(vpRxyzVector(M_PI/2, 0.0, -M_PI/2))
);

// PIMPL: ViSP/RealSense 타입을 헤더에서 완전히 숨김
struct VisualServo::Impl {
    vpCameraParameters  cam;
    vpHomogeneousMatrix rMc;
};

static VisualServo::Pose toRBDLPose(const vpHomogeneousMatrix& M)
{
    VisualServo::Pose p;
    p.m_position[0] = M[0][3];
    p.m_position[1] = M[1][3];
    p.m_position[2] = M[2][3];
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            p.m_rotation(r, c) = M[r][c];
    p.m_rotation_rpy.setZero();
    return p;
}

VisualServo::VisualServo(const std::string& calibDir)
    : m_calibDir(calibDir), m_pImpl(std::make_unique<Impl>())
{
}

VisualServo::~VisualServo()
{
    Stop();
}

bool VisualServo::Init()
{
    // 1. camera.xml 로드
    vpXmlParserCamera parser;
    std::string xmlPath = m_calibDir + "/camera.xml";
    if (parser.parse(m_pImpl->cam, xmlPath, "RealSense_color",
                     vpCameraParameters::perspectiveProjWithDistortion) != vpXmlParserCamera::SEQUENCE_OK)
    {
        printf("[VisualServo] ERROR: Failed to load camera.xml from %s\n", xmlPath.c_str());
        return false;
    }
    printf("[VisualServo] camera.xml loaded. px=%.1f py=%.1f\n",
           m_pImpl->cam.get_px(), m_pImpl->cam.get_py());

    // 2. rPc.yaml 로드
    std::string rpcPath = m_calibDir + "/rPc.yaml";
    try {
        vpPoseVector rPc;
        vpArray2D<double>::loadYAML(rpcPath, rPc);
        m_pImpl->rMc = vpHomogeneousMatrix(rPc);
    } catch (const std::exception& e) {
        printf("[VisualServo] ERROR: Failed to load rPc.yaml: %s\n", e.what());
        return false;
    }
    printf("[VisualServo] rPc.yaml loaded. t=[%.3f, %.3f, %.3f]\n",
           m_pImpl->rMc[0][3], m_pImpl->rMc[1][3], m_pImpl->rMc[2][3]);

    // 3. RealSense 연결 확인 (device query only — no stream open, avoids errno=16 on busy device)
    try {
        rs2::context ctx;
        auto devices = ctx.query_devices();
        if (devices.size() == 0) {
            printf("[VisualServo] ERROR: No RealSense camera found\n");
            return false;
        }
        printf("[VisualServo] RealSense found: %s\n",
               devices[0].get_info(RS2_CAMERA_INFO_NAME));
    } catch (const std::exception& e) {
        printf("[VisualServo] ERROR: Failed to query RealSense: %s\n", e.what());
        return false;
    }

    m_initialized = true;
    return true;
}

void VisualServo::Start()
{
    if (!m_initialized) {
        printf("[VisualServo] ERROR: Call Init() before Start()\n");
        return;
    }
    if (m_running.load()) return;
    m_running   = true;
    m_valid     = false;
    m_lostCount = 0;
    m_state     = State::TRACKING;
    printf("[VisualServo] Started\n");
}

void VisualServo::Stop()
{
    m_running = false;
    m_valid   = false;
    m_state   = State::IDLE;
    printf("[VisualServo] Stopped\n");
}

bool VisualServo::GetGoalPose(Pose& out) const
{
    if (m_state.load() == State::SINGULARITY) return false;
    if (!m_valid.load()) return false;
    out = m_buf[m_latest.load()];
    return true;
}

void VisualServo::NotifySingularity()
{
    if (m_state.load() == State::TRACKING || m_state.load() == State::TAG_LOST)
    {
        m_state = State::SINGULARITY;
        printf("[VisualServo] WARN: Singularity — switching to gravity compensation\n");
    }
}

void VisualServo::Loop()
{
    // pipe.start() is intentionally called here (not in Init()) so that it runs
    // from the non-RT proc_visual_servo thread. This keeps RealSense USB worker
    // threads at non-RT priority and prevents EtherCAT Sync Manager watchdog.
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGBA8, 30);

    try {
        pipe.start(cfg);
    } catch (const std::exception& e) {
        printf("[VisualServo] ERROR: Camera open failed: %s\n", e.what());
        m_running = false;
        m_state   = State::IDLE;
        return;
    }
    printf("[VisualServo] Camera open, loop start\n");

    vpImage<vpRGBa>        I_color(480, 640);
    vpImage<unsigned char> I(480, 640);
    vpDetectorAprilTag detector(vpDetectorAprilTag::TAG_36h11);
    detector.setAprilTagQuadDecimate(1.0);
    detector.setAprilTagPoseEstimationMethod(vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS);
    detector.setAprilTagNbThreads(1);
    const double tagSize = 0.09;

    rs2::frameset frames;
    bool firstLoop = true;
    int frameCount = 0;

    while (m_running.load())
    {
        if (firstLoop) printf("[VS Loop] acquiring...\n");
        if (!pipe.try_wait_for_frames(&frames, 200))
            continue;
        rs2::video_frame color = frames.get_color_frame();
        if (!color) { printf("[VS Loop] ERROR: invalid frame\n"); continue; }
        memcpy(I_color.bitmap, color.get_data(), 640 * 480 * 4);
        if (firstLoop) printf("[VS Loop] acquired\n");

        for (unsigned int i = 0; i < I_color.getHeight() * I_color.getWidth(); i++) {
            const vpRGBa& px = I_color.bitmap[i];
            I.bitmap[i] = (unsigned char)(0.299f*px.R + 0.587f*px.G + 0.114f*px.B);
        }
        if (firstLoop) printf("[VS Loop] gray converted\n");

        if (m_state.load() == State::SINGULARITY) { firstLoop = false; continue; }

        std::vector<vpHomogeneousMatrix> cMo_vec;
        if (firstLoop) printf("[VS Loop] detecting...\n");
        detector.detect(I, tagSize, m_pImpl->cam, cMo_vec);
        if (firstLoop) printf("[VS Loop] detected %zu tags\n", cMo_vec.size());

        if (cMo_vec.empty())
        {
            m_lostCount++;
            if (m_lostCount == 1)
                printf("[VisualServo] WARN: Tag lost — holding last pose\n");
            if (m_state.load() == State::TRACKING)
                m_state = State::TAG_LOST;
            continue;
        }

        if (m_state.load() == State::TAG_LOST)
        {
            printf("[VisualServo] Tag recovered after %d frames\n", m_lostCount);
            m_state = State::TRACKING;
        }
        m_lostCount = 0;

        vpHomogeneousMatrix goal = m_pImpl->rMc * cMo_vec[0] * s_Toffset;

        // 지수 이동 평균 필터 (α=0.15) — goal 노이즈 감소
        static vpHomogeneousMatrix goalFiltered;
        static bool filterInit = false;
        const double alpha = 0.15;
        if (!filterInit) { goalFiltered = goal; filterInit = true; }
        for (int r = 0; r < 3; r++) {
            goalFiltered[r][3] = alpha * goal[r][3] + (1.0 - alpha) * goalFiltered[r][3];
            for (int c = 0; c < 3; c++)
                goalFiltered[r][c] = alpha * goal[r][c] + (1.0 - alpha) * goalFiltered[r][c];
        }

        if (firstLoop) printf("[VS Loop] goal t=[%.3f, %.3f, %.3f]\n", goal[0][3], goal[1][3], goal[2][3]);
        if (++frameCount % 30 == 0) printf("[VS] goal t=[%.3f, %.3f, %.3f]\n", goalFiltered[0][3], goalFiltered[1][3], goalFiltered[2][3]);

        int next = 1 - m_latest.load();
        m_buf[next] = toRBDLPose(goalFiltered);
        m_latest.store(next);
        m_valid.store(true);
        firstLoop = false;
    }

    pipe.stop();
}
