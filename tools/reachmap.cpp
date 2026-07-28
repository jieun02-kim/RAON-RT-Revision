/*****************************************************************************
*   Name: reachmap.cpp
*   Affiliation: RAIMLAB @ Myongji University
*   Description: [kv260-merge] Offline reach map for the ready-seed IK.
*
*   WHY THIS EXISTS
*   ---------------
*   Approach success has been judged all along by an 8-corner probe count
*   ("6/8"), which is NOT a reachability measure: those are the corners of the
*   perception box, radius-clamped, and a target can sit inside a box whose
*   corners all pass yet still be refused (E27's mustard) or sit outside every
*   corner and pass. Worse, mustard was found to flip arm branch on a 2 mm
*   change of target (x 0.828 -> 0.830), so the interesting structure is a
*   BOUNDARY in the workspace and no eight points can show where it runs.
*
*   This sweeps a grid over the table plane and asks SolveReadyIK itself for a
*   verdict at every cell, writing CSV for plotting.
*
*   WHY IT LINKS THE CONTROLLER INSTEAD OF REIMPLEMENTING IT
*   -------------------------------------------------------
*   A second copy of the ladder / warm-start / fold / branch / tilt logic would
*   be free to drift from the one the robot runs, and a map that disagrees with
*   the robot is worse than no map. So this links FullDynControllerRT.o and
*   calls the very same SolveReadyIK. If the solver changes, the map changes
*   with it, and `--targets` replays field coordinates so the agreement can be
*   checked against real logs rather than assumed.
*
*   WHAT THE MAP DOES NOT KNOW
*   --------------------------
*   RBDL carries no geometry, so there is NO collision checking of any kind:
*   the table, the camera rig, the objects and the robot's own links are all
*   absent. A cell marked PASS means "the IK accepts this point", never "the
*   arm can safely go there". Self-collision and table strikes remain a MuJoCo
*   job. Marks drawn on the plot (--mark) are annotation only.
*****************************************************************************/

#include "FullDynControllerRT.h"
#include "WorkspaceBox.h"        // not ROS2PickBridge.h: that one pulls rclcpp

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>
#include <unistd.h>

using RigidBodyDynamics::Math::Vector3d;
using RigidBodyDynamics::Math::VectorNd;

namespace {

const char* VerdictName(CControllerFullDynamicsRT::eIkVerdict aeV)
{
    switch (aeV)
    {
        case CControllerFullDynamicsRT::eIkPass:       return "PASS";
        case CControllerFullDynamicsRT::eIkNoSolution: return "NO_SOLUTION";
        case CControllerFullDynamicsRT::eIkBranch:     return "BRANCH";
        case CControllerFullDynamicsRT::eIkLimits:     return "LIMITS";
        case CControllerFullDynamicsRT::eIkTilt:       return "TILT";
        case CControllerFullDynamicsRT::eIkNoSeed:     return "NO_SEED";
    }
    return "?";
}

struct Mark { double x, y, z; std::string label; };

struct Options {
    std::string strURDF  = "App/Indy7/indy7.urdf";
    std::string strSeed;                       // default $HOME/.indy7_ready_seed
    std::string strOut   = "reachmap.csv";
    std::string strLog   = "reachmap.log";
    std::string strTargets;                    // replay mode
    double      dX0 = kv260::WS_BOX[0];
    double      dX1 = kv260::WS_BOX[1];
    double      dY0 = kv260::WS_BOX[2];
    double      dY1 = kv260::WS_BOX[3];
    double      dStep = 0.01;
    double      dRMax = kv260::WS_RMAX_XY;
    bool        bClampR = false;               // clamp to rmax instead of skipping
    std::vector<double> vZ = { 0.18 };
    std::vector<Mark>   vMarks;
};

bool ParsePair(const char* asz, double& ad0, double& ad1)
{ return sscanf(asz, "%lf,%lf", &ad0, &ad1) == 2; }

void ParseList(const char* asz, std::vector<double>& av)
{
    av.clear();
    const char* p = asz;
    while (*p)
    {
        char* pEnd = NULL;
        const double d = strtod(p, &pEnd);
        if (pEnd == p) break;
        av.push_back(d);
        p = pEnd;
        while (*p == ',' || *p == ' ') p++;
    }
}

void Usage(const char* aszProg)
{
    fprintf(stderr,
        "usage: %s [options]\n"
        "  --urdf PATH        URDF (default App/Indy7/indy7.urdf)\n"
        "  --seed PATH        ready-seed file (default $HOME/.indy7_ready_seed)\n"
        "  --out PATH         CSV output (default reachmap.csv)\n"
        "  --log PATH         solver chatter sink (default reachmap.log)\n"
        "  --x a,b            x range [m] (default from DEF_BOX)\n"
        "  --y a,b            y range [m] (default from DEF_BOX)\n"
        "  --z z[,z...]       scan height(s) [m] (default 0.18)\n"
        "  --step S           grid step [m] (default 0.01)\n"
        "  --rmax R           xy radius cap [m] (default DEF_RMAX_XY)\n"
        "  --clamp-r          clamp cells past rmax onto it (bridge behaviour)\n"
        "                     instead of leaving them out of the sweep\n"
        "  --mark x,y,z,label annotate a point (repeatable)\n"
        "  --targets PATH     replay mode: one 'x y z [label]' per line,\n"
        "                     no grid — for checking the map against field logs\n",
        aszProg);
}

bool ParseArgs(int argc, char** argv, Options& aOpt)
{
    for (int i = 1; i < argc; i++)
    {
        const std::string strFlag = argv[i];
        if (strFlag == "--clamp-r") { aOpt.bClampR = true; continue; }
        if (i + 1 >= argc) { Usage(argv[0]); return false; }
        const char* szVal = argv[++i];

        if      (strFlag == "--urdf")    aOpt.strURDF    = szVal;
        else if (strFlag == "--seed")    aOpt.strSeed    = szVal;
        else if (strFlag == "--out")     aOpt.strOut     = szVal;
        else if (strFlag == "--log")     aOpt.strLog     = szVal;
        else if (strFlag == "--targets") aOpt.strTargets = szVal;
        else if (strFlag == "--step")    aOpt.dStep      = atof(szVal);
        else if (strFlag == "--rmax")    aOpt.dRMax      = atof(szVal);
        else if (strFlag == "--z")       ParseList(szVal, aOpt.vZ);
        else if (strFlag == "--x")
        { if (!ParsePair(szVal, aOpt.dX0, aOpt.dX1)) { Usage(argv[0]); return false; } }
        else if (strFlag == "--y")
        { if (!ParsePair(szVal, aOpt.dY0, aOpt.dY1)) { Usage(argv[0]); return false; } }
        else if (strFlag == "--mark")
        {
            Mark m; char acLabel[128] = "";
            if (sscanf(szVal, "%lf,%lf,%lf,%127s", &m.x, &m.y, &m.z, acLabel) < 3)
            { Usage(argv[0]); return false; }
            m.label = acLabel;
            aOpt.vMarks.push_back(m);
        }
        else { Usage(argv[0]); return false; }
    }
    if (aOpt.dX1 < aOpt.dX0) std::swap(aOpt.dX0, aOpt.dX1);
    if (aOpt.dY1 < aOpt.dY0) std::swap(aOpt.dY0, aOpt.dY1);
    if (aOpt.dStep <= 0.0 || aOpt.vZ.empty()) { Usage(argv[0]); return false; }
    if (aOpt.strSeed.empty())
    {
        const char* szHome = getenv("HOME");
        aOpt.strSeed = std::string(szHome ? szHome : ".") + "/.indy7_ready_seed";
    }
    return true;
}

bool LoadSeed(const std::string& astrPath, VectorNd& aq, unsigned int auDOF)
{
    FILE* pF = fopen(astrPath.c_str(), "r");
    if (pF == NULL) return false;
    double adQ[8]; unsigned int nRead = 0;
    while (nRead < 8 && fscanf(pF, "%lf", &adQ[nRead]) == 1) nRead++;
    fclose(pF);
    if (nRead < auDOF) return false;
    aq.resize(auDOF);
    for (unsigned int i = 0; i < auDOF; i++) aq[i] = adQ[i];
    return true;
}

// The app's init-time probe set, reproduced so the map header can report the
// same n/8 the robot prints — the two numbers being comparable is the point.
std::vector<Vector3d> BuildProbes()
{
    const double* pdBox = kv260::WS_BOX;
    const double  dRMax = kv260::WS_RMAX_XY;
    std::vector<Vector3d> v;
    for (int ix = 0; ix < 2; ix++)
        for (int iy = 0; iy < 2; iy++)
            for (int iz = 0; iz < 2; iz++)
            {
                double dX = pdBox[ix], dY = pdBox[2 + iy];
                const double dR = std::sqrt(dX * dX + dY * dY);
                if (dR > dRMax) { dX *= dRMax / dR; dY *= dRMax / dR; }
                v.push_back(Vector3d(dX, dY, pdBox[4 + iz]));
            }
    return v;
}

} // namespace

int main(int argc, char** argv)
{
    Options stOpt;
    if (!ParseArgs(argc, argv, stOpt)) return 2;

    // Keep the terminal readable: the solver logs several lines per cell and a
    // sweep is thousands of cells. Progress goes to the ORIGINAL stderr, kept
    // via dup() before the redirect.
    FILE* pProgress = fdopen(dup(fileno(stderr)), "w");
    if (freopen(stOpt.strLog.c_str(), "w", stdout) == NULL)
    { fprintf(pProgress, "cannot write %s\n", stOpt.strLog.c_str()); return 1; }
    if (freopen(stOpt.strLog.c_str(), "a", stderr) == NULL) return 1;

    const unsigned int uDOF = 6;
    CControllerFullDynamicsRT cCtrl(stOpt.strURDF, uDOF);
    if (!cCtrl.Init())
    {
        fprintf(pProgress, "URDF load failed: %s\n", stOpt.strURDF.c_str());
        return 1;
    }

    VectorNd vqSeed;
    if (!LoadSeed(stOpt.strSeed, vqSeed, uDOF))
    {
        fprintf(pProgress, "no usable seed at %s — record HOME first\n",
                stOpt.strSeed.c_str());
        return 1;
    }

    // Same entry point the app uses at init, so q_ready here IS q_ready there
    // (fold, limit check and the 8-probe coverage count all included).
    const std::vector<Vector3d> vProbes = BuildProbes();
    const Vector3d vCenter(0.5 * (kv260::WS_BOX[0] + kv260::WS_BOX[1]),
                           0.5 * (kv260::WS_BOX[2] + kv260::WS_BOX[3]),
                           0.5 * (kv260::WS_BOX[4] + kv260::WS_BOX[5]));
    if (!cCtrl.ComputeReadySeed(vProbes, vCenter, &vqSeed))
    {
        fprintf(pProgress, "seed rejected (limits?) — see %s\n", stOpt.strLog.c_str());
        return 1;
    }
    const VectorNd vqReady = cCtrl.GetReadyQ();

    // Park the model at q_ready. SolveReadyIK folds its answer toward the LIVE
    // posture and reports the travel from it; with m_Q left at construction
    // garbage that output would be meaningless (and could be NaN). "Arm sitting
    // at ready" is also the assumption the map is drawn under.
    cCtrl.Enable(TRUE);
    {
        std::vector<double> vPos(uDOF), vVel(uDOF, 0.0), vTor(uDOF, 0.0), vOut(uDOF, 0.0);
        for (unsigned int i = 0; i < uDOF; i++) vPos[i] = vqReady[i];
        cCtrl.Update(vPos, vVel, vTor, vOut);
    }

    FILE* pOut = fopen(stOpt.strOut.c_str(), "w");
    if (pOut == NULL)
    { fprintf(pProgress, "cannot write %s\n", stOpt.strOut.c_str()); return 1; }

    // Header comments carry the provenance: a map is only readable next to the
    // anchor that produced it.
    fprintf(pOut, "# reachmap urdf=%s seed=%s\n",
            stOpt.strURDF.c_str(), stOpt.strSeed.c_str());
    fprintf(pOut, "# q_ready = [%.9f %.9f %.9f %.9f %.9f %.9f]\n",
            vqReady[0], vqReady[1], vqReady[2], vqReady[3], vqReady[4], vqReady[5]);
    fprintf(pOut, "# rmax=%.3f clamp_r=%d step=%.4f\n",
            stOpt.dRMax, stOpt.bClampR ? 1 : 0, stOpt.dStep);
    for (size_t m = 0; m < stOpt.vMarks.size(); m++)
        fprintf(pOut, "# mark %.6f %.6f %.6f %s\n", stOpt.vMarks[m].x,
                stOpt.vMarks[m].y, stOpt.vMarks[m].z, stOpt.vMarks[m].label.c_str());
    // seed/cands are multi-start only; the baseline map has neither, which is
    // why reachmap_compare.py treats them as optional.
    fprintf(pOut, "x,y,z,verdict,w,tilt_deg,gap_rad,gap_axis,pos_err_mm,"
                  "solves,ms,seed,cands,label\n");

    // One cell: ask the robot's own solver, then record what it did.
    struct Cell { double x, y, z; std::string label; };
    std::vector<Cell> vCells;
    if (!stOpt.strTargets.empty())
    {
        FILE* pT = fopen(stOpt.strTargets.c_str(), "r");
        if (pT == NULL)
        { fprintf(pProgress, "cannot read %s\n", stOpt.strTargets.c_str()); return 1; }
        char acLine[512];
        while (fgets(acLine, sizeof(acLine), pT) != NULL)
        {
            if (acLine[0] == '#' || acLine[0] == '\n') continue;
            Cell c; char acLabel[128] = "";
            const int n = sscanf(acLine, "%lf %lf %lf %127s", &c.x, &c.y, &c.z, acLabel);
            if (n >= 3) { c.label = acLabel; vCells.push_back(c); }
        }
        fclose(pT);
    }
    else
    {
        for (size_t iz = 0; iz < stOpt.vZ.size(); iz++)
            for (double y = stOpt.dY0; y <= stOpt.dY1 + 1e-9; y += stOpt.dStep)
                for (double x = stOpt.dX0; x <= stOpt.dX1 + 1e-9; x += stOpt.dStep)
                {
                    Cell c; c.x = x; c.y = y; c.z = stOpt.vZ[iz];
                    const double dR = std::sqrt(x * x + y * y);
                    if (dR > stOpt.dRMax)
                    {
                        if (!stOpt.bClampR) continue;   // bridge would reject it
                        c.x = x * stOpt.dRMax / dR;
                        c.y = y * stOpt.dRMax / dR;
                    }
                    vCells.push_back(c);
                }
    }

    fprintf(pProgress, "reachmap: %zu cells, q_ready = [%.3f %.3f %.3f %.3f %.3f %.3f]\n",
            vCells.size(), vqReady[0], vqReady[1], vqReady[2],
            vqReady[3], vqReady[4], vqReady[5]);

    int anCount[6] = {0, 0, 0, 0, 0, 0};
    double dMsMax = 0.0, dMsSum = 0.0;
    for (size_t i = 0; i < vCells.size(); i++)
    {
        const Cell& c = vCells[i];
        VectorNd vqSol;
        double dPosErr = 0.0, dDq = 0.0;
        int    nDqAxis = -1;
        cCtrl.SolveReadyIK(Vector3d(c.x, c.y, c.z), vqSol, dPosErr, dDq, nDqAxis);
        const CControllerFullDynamicsRT::IkDiag& d = cCtrl.GetLastIkDiag();

        anCount[(int)d.eVerdict]++;
        dMsSum += d.dMs;
        if (d.dMs > dMsMax) dMsMax = d.dMs;

        fprintf(pOut, "%.4f,%.4f,%.4f,%s,%.2f,%.2f,%.4f,%d,%.2f,%d,%.3f,"
                      "%d,%d,%s\n",
                c.x, c.y, c.z, VerdictName(d.eVerdict), d.dW, d.dTiltDeg,
                d.dBranchGap, d.nBranchAxis, d.dPosErrM * 1e3, d.nSolves, d.dMs,
                d.nSeed, d.nCandidates, c.label.c_str());

        if ((i % 200) == 0 || i + 1 == vCells.size())
        {
            fprintf(pProgress, "\r  %zu/%zu  pass %d  branch %d  tilt %d  "
                    "unreach %d  limits %d", i + 1, vCells.size(),
                    anCount[0], anCount[2], anCount[4], anCount[1], anCount[3]);
            fflush(pProgress);
        }
    }
    fclose(pOut);

    fprintf(pProgress, "\nwrote %s\n", stOpt.strOut.c_str());
    fprintf(pProgress, "  PASS %d | BRANCH %d | TILT %d | NO_SOLUTION %d | "
            "LIMITS %d | NO_SEED %d\n",
            anCount[0], anCount[2], anCount[4], anCount[1], anCount[3], anCount[5]);
    fprintf(pProgress, "  solve time: mean %.2f ms, max %.2f ms  "
            "(the 1 kHz budget is 1.00 ms — see merge.md E25)\n",
            vCells.empty() ? 0.0 : dMsSum / (double)vCells.size(), dMsMax);
    fprintf(pProgress, "  NOTE: no collision model. PASS means the IK accepts "
            "the point, not that the arm can safely reach it.\n");
    return 0;
}
