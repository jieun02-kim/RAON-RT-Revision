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
