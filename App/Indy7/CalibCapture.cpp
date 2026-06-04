#include "CalibCapture.h"
#include <sstream>
#include <cstdio>
#include <sys/stat.h>

CalibCapture::CalibCapture(const std::string& outDir)
    : m_outDir(outDir), m_count(1)
{
    mkdir(outDir.c_str(), 0777);
}

void CalibCapture::Capture(const CControllerFullDynamicsRT::Pose& pose)
{
    vpHomogeneousMatrix rMe = ToMatrix(pose);
    vpPoseVector rPe(rMe);

    std::stringstream ss;
    ss << m_outDir << "/pose_rPe_" << m_count << ".yaml";
    rPe.saveYAML(ss.str(), rPe);

    printf("[CalibCapture] #%d saved: t=[%.4f, %.4f, %.4f] → %s\n",
           m_count,
           pose.m_position[0], pose.m_position[1], pose.m_position[2],
           ss.str().c_str());
    m_count++;
}

void CalibCapture::Reset()
{
    m_count = 0;
}

vpHomogeneousMatrix CalibCapture::ToMatrix(const CControllerFullDynamicsRT::Pose& pose)
{
    vpHomogeneousMatrix M;
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            M[r][c] = pose.m_rotation(r, c);
    M[0][3] = pose.m_position[0];
    M[1][3] = pose.m_position[1];
    M[2][3] = pose.m_position[2];
    M[3][3] = 1.0;
    return M;
}
