/*****************************************************************************
*	Name: gate2b_bridge_test.cpp
*	[kv260-merge] Gate 2b harness — drives CROS2PickBridge standalone
*	(no EtherCAT / no robot), reading single-letter commands from stdin:
*	    p        object menu (same as the app's 'p' key)
*	    0-9      menu selection
*	    v        approach request → statistics gate
*	    q        quit
*	A poller thread consumes gated goals exactly like the RT loop would and
*	prints "[HARNESS] GOAL CONSUMED: ...". Build:  make gate2b_test
*	Run under tools/test_gate2b_bridge.py for the scripted PASS/FAIL suite.
*****************************************************************************/
#include <atomic>
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

#include "ROS2PickBridge.h"

int main()
{
    // stdout goes through a pipe under the python driver — line-buffer it so
    // every printf (bridge threads included) is visible immediately.
    setvbuf(stdout, nullptr, _IOLBF, 0);

    CROS2PickBridge cBridge;
    // Harness runs against the synthetic feeder / placeholder TF — the tight
    // in-app default box would reject everything. Wide-open on purpose.
    cBridge.SetWorkspaceBox(-2.0, 2.0, -2.0, 2.0, -2.0, 2.0);

    if (!cBridge.Init())
    {
        printf("[HARNESS] FATAL: bridge init failed\n");
        return 1;
    }

    std::atomic<bool> bStop{false};
    std::thread thPoller([&]()
    {
        // stand-in for the 1 kHz RT consumer
        CROS2PickBridge::Goal stGoal;
        // [gui] fake state so the console has something to gate on: servo/
        // ctrl/grav/home true (most buttons enabled in a dry run), joint1
        // waving so /joint_states + the 3D pane are visibly LIVE, not latched.
        CROS2PickBridge::RobotState stFake{};
        stFake.nDof = 6;
        stFake.dQ[1] = -1.5707963;
        stFake.bServo = stFake.bCtrl = stFake.bGrav = stFake.bHome = true;
        // [gui] fake gains readback (the app's stock ladder) so the console's
        // admin tab lights up; PopGains below feeds applied sets back in,
        // mirroring what the real RT loop does via the controller.
        stFake.bGains = true;
        stFake.nMode = 2;   // ComputedTorque — the jog gate's happy path
        const double adKp0[6] = {800, 600, 400, 200, 150, 100};
        const double adKd0[6] = {80, 60, 40, 20, 15, 10};
        for (int i = 0; i < 6; i++)
        {
            stFake.dKp[i] = adKp0[i];
            stFake.dKd[i] = adKd0[i];
            stFake.dKf[i] = 0.0;
        }
        uint32_t uJogSeqSeen = 0;
        int nTick = 0;
        while (!bStop.load())
        {
            if (cBridge.PopGoal(stGoal))
            {
                printf("[HARNESS] GOAL CONSUMED: class=%s xyz=(%.3f, %.3f, %.3f) "
                       "std_mm=(%.1f, %.1f, %.1f) n=%d\n",
                       stGoal.szClass, stGoal.dX, stGoal.dY, stGoal.dZ,
                       stGoal.dStdMM[0], stGoal.dStdMM[1], stGoal.dStdMM[2],
                       stGoal.nSamples);
            }
            // [gui] the two operator-console paths, driven from the same
            // thread the RT loop would use.
            char cKey = ' ';
            if (cBridge.PopKey(cKey))
                printf("[HARNESS] REMOTE KEY: '%c'\n", cKey);
            CROS2PickBridge::JogCmd stJog;
            uint32_t uJogSeq = 0;
            if (cBridge.PeekJog(stJog, uJogSeq) && uJogSeq != uJogSeqSeen)
            {
                uJogSeqSeen = uJogSeq;
                // mirror the RT walk instantly: the fake joint snaps to the
                // target so the console's actual column shows the effect
                if (stJog.nAxis >= 0 && stJog.nAxis < 6)
                    stFake.dQ[stJog.nAxis] = stJog.dQ;
                printf("[HARNESS] JOG CONSUMED: axis=%d q=%.4f rad\n",
                       stJog.nAxis, stJog.dQ);
            }
            double adG[3 * CROS2PickBridge::GAINS_DOF];
            if (cBridge.PopGains(adG))
            {
                for (int i = 0; i < CROS2PickBridge::GAINS_DOF; i++)
                {
                    stFake.dKp[i] = adG[i];
                    stFake.dKd[i] = adG[CROS2PickBridge::GAINS_DOF + i];
                    stFake.dKf[i] = adG[2 * CROS2PickBridge::GAINS_DOF + i];
                }
                printf("[HARNESS] GAINS CONSUMED: Kp0=%.1f Kd0=%.1f Kf0=%.2f\n",
                       stFake.dKp[0], stFake.dKd[0], stFake.dKf[0]);
            }
            stFake.dQ[1] = -1.5707963 + 0.2 * (double)((nTick++ % 20) - 10) / 10.0;
            cBridge.PushState(stFake);

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });

    printf("[HARNESS] ready — commands: p, 0-9, v, q\n");
    char acLine[64];
    while (fgets(acLine, sizeof(acLine), stdin) != nullptr)
    {
        const char c = acLine[0];
        if (c == 'q')
            break;
        else if (c == 'p')
            cBridge.RequestMenu();
        else if (c == 'v')
            cBridge.RequestApproach();
        else if (c >= '0' && c <= '9')
        {
            if (!cBridge.FeedDigit(c))
                printf("[HARNESS] digit ignored (no menu open)\n");
        }
        else if (c == 's')
            printf("[HARNESS] locked=%d\n", (int)cBridge.HasLockedTarget());
    }

    bStop = true;
    thPoller.join();
    cBridge.DeInit();
    printf("[HARNESS] exit\n");
    return 0;
}
