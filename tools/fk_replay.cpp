/*****************************************************************************
*   Name: fk_replay.cpp
*   [kv260-merge] Joint-log → task-space converter for the per-approach
*   trajectory plots (2026-07-30).
*
*   Reads a proc_logger DataLog CSV (per-cycle q / q_ref / tcp / goal) and
*   appends six columns:
*       ref_x, ref_y, ref_z          FK of q_ref  — where the trajectory
*                                    REFERENCE was each cycle
*       act_fk_x, act_fk_y, act_fk_z FK of q      — offline recomputation of
*                                    the actual TCP
*
*   Why link FullDynControllerRT.o instead of doing FK in the plot script:
*   the reach-map lesson. Kinematics reimplemented next to the robot drifts,
*   and a plot that disagrees with the robot is worse than no plot. ToolPosAt
*   uses the same model object and the same tcp_local_point the robot runs.
*
*   act_fk_* is deliberately redundant with the logged tcp_* (which the RT
*   loop computed with the same model): the plot watcher compares them and
*   flags any mismatch — a free continuous cross-check that this offline
*   pipeline and the robot still agree.
*
*   Usage:  fk_replay.out <urdf> <in.csv> [out.csv]
*           (out defaults to <in stem>_fk.csv; URDF must be an absolute path
*            when cwd differs — the reachmap relative-default trap.)
*****************************************************************************/
#include "FullDynControllerRT.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

using RigidBodyDynamics::Math::Vector3d;
using RigidBodyDynamics::Math::VectorNd;

namespace {

// Split one CSV line in place; returns pointers into the buffer.
int SplitCSV(char* apLine, std::vector<char*>& avOut)
{
    avOut.clear();
    for (char* p = strtok(apLine, ",\r\n"); p != NULL;
         p = strtok(NULL, ",\r\n"))
        avOut.push_back(p);
    return (int)avOut.size();
}

} // namespace

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        fprintf(stderr,
                "usage: %s <urdf> <DataLog.csv> [out.csv]\n"
                "       appends ref_x/y/z (FK of q_ref) and act_fk_x/y/z "
                "(FK of q)\n", argv[0]);
        return 2;
    }
    const std::string strURDF = argv[1];
    const std::string strIn   = argv[2];
    std::string strOut;
    if (argc > 3)
        strOut = argv[3];
    else
    {
        strOut = strIn;
        const size_t uDot = strOut.rfind(".csv");
        strOut = (uDot == std::string::npos) ? strOut + "_fk.csv"
                                             : strOut.substr(0, uDot) + "_fk.csv";
    }

    CControllerFullDynamicsRT cCtrl(strURDF, 6);
    if (!cCtrl.Init())
    {
        fprintf(stderr, "fk_replay: URDF load failed: %s\n", strURDF.c_str());
        return 1;
    }

    FILE* pIn = fopen(strIn.c_str(), "r");
    if (pIn == NULL)
    {
        fprintf(stderr, "fk_replay: cannot open %s\n", strIn.c_str());
        return 1;
    }

    // Header: locate q0..q5 and q_ref0..q_ref5 by NAME — the DataLog layout
    // has grown before and may again.
    char acLine[4096];
    if (fgets(acLine, sizeof(acLine), pIn) == NULL)
    {
        fprintf(stderr, "fk_replay: empty file\n");
        fclose(pIn);
        return 1;
    }
    char acHeaderKeep[4096];
    snprintf(acHeaderKeep, sizeof(acHeaderKeep), "%s", acLine);

    std::vector<char*> vCols;
    SplitCSV(acLine, vCols);
    int anQ[6], anQRef[6];
    for (int i = 0; i < 6; i++) { anQ[i] = -1; anQRef[i] = -1; }
    for (int c = 0; c < (int)vCols.size(); c++)
        for (int i = 0; i < 6; i++)
        {
            char acName[16];
            snprintf(acName, sizeof(acName), "q%d", i);
            if (strcmp(vCols[c], acName) == 0) anQ[i] = c;
            snprintf(acName, sizeof(acName), "q_ref%d", i);
            if (strcmp(vCols[c], acName) == 0) anQRef[i] = c;
        }
    for (int i = 0; i < 6; i++)
        if (anQ[i] < 0 || anQRef[i] < 0)
        {
            fprintf(stderr, "fk_replay: header lacks q%d/q_ref%d — not a "
                    "DataLog CSV?\n", i, i);
            fclose(pIn);
            return 1;
        }

    FILE* pOut = fopen(strOut.c_str(), "w");
    if (pOut == NULL)
    {
        fprintf(stderr, "fk_replay: cannot write %s\n", strOut.c_str());
        fclose(pIn);
        return 1;
    }
    // header passthrough + new columns
    char* pNL = strpbrk(acHeaderKeep, "\r\n");
    if (pNL != NULL) *pNL = '\0';
    fprintf(pOut, "%s,ref_x,ref_y,ref_z,act_fk_x,act_fk_y,act_fk_z\n",
            acHeaderKeep);

    VectorNd vqRef(6), vqAct(6);
    long nRows = 0;
    char acKeep[4096];
    while (fgets(acLine, sizeof(acLine), pIn) != NULL)
    {
        snprintf(acKeep, sizeof(acKeep), "%s", acLine);
        if (SplitCSV(acLine, vCols) < 2)
            continue;                      // torn last line
        bool bOK = true;
        for (int i = 0; i < 6 && bOK; i++)
        {
            if (anQ[i] >= (int)vCols.size() || anQRef[i] >= (int)vCols.size())
                { bOK = false; break; }
            vqAct[i] = atof(vCols[anQ[i]]);
            vqRef[i] = atof(vCols[anQRef[i]]);
        }
        if (!bOK) continue;

        const Vector3d vRef = cCtrl.ToolPosAt(vqRef);
        const Vector3d vAct = cCtrl.ToolPosAt(vqAct);

        pNL = strpbrk(acKeep, "\r\n");
        if (pNL != NULL) *pNL = '\0';
        fprintf(pOut, "%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", acKeep,
                vRef[0], vRef[1], vRef[2], vAct[0], vAct[1], vAct[2]);
        nRows++;
    }
    fclose(pIn);
    fclose(pOut);
    printf("fk_replay: %ld rows → %s\n", nRows, strOut.c_str());
    return 0;
}
