/*
 * Gate 0 smoke test — RT-POSIX periodic task jitter on KV260 (kv260-merge branch)
 *
 * Validates the /opt/rt_posix install that RAON-RT's RT tasks depend on:
 * create_rt_task -> set_task_period(1ms) -> start_task -> wait_next_period loop,
 * measuring period jitter with read_timer() (CLOCK_MONOTONIC ns).
 *
 * Build: gcc -O2 -Wall -I/opt/rt_posix/include -o gate0_rtposix_test \
 *            gate0_rtposix_test.c -L/opt/rt_posix/lib -lrtposix -lpthread -lrt -lm
 * Run:   sudo ./gate0_rtposix_test [cycles=5000] [period_ns=1000000] [prio=80] [v]
 *        (non-root without rtprio ulimit -> NRT fallback, API mechanics only;
 *         4th arg enables the RT-POSIX low-level logger for diagnosis)
 *
 * NOTE: wait_next_period()'s overrun detection in RT-POSIX compares the wrong
 *       direction at equal tv_sec (deadline-in-future counted as overrun), so
 *       this test passes NULL and judges purely by measured dt statistics.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/mman.h>
#include "posix_rt.h"

typedef struct {
    long    cycles;
    RTTIME  period;
    volatile int done;
    int     rt_mode;
    double  max_err_us, sum_err_us;
    RTTIME  min_dt, max_dt;
    UINT64  overruns;
} CTX;

static CTX        g_ctx;
static POSIX_TASK g_task;

static void periodic_proc(void* arg)
{
    CTX* c = (CTX*)arg;

    wait_next_period(NULL);             /* align to the first period edge */
    RTTIME prev = read_timer();
    c->min_dt = (RTTIME)-1;

    for (long i = 0; i < c->cycles; i++) {
        wait_next_period(NULL);
        RTTIME now = read_timer();
        RTTIME dt  = now - prev;
        prev = now;

        double err  = ((double)dt - (double)c->period) / 1000.0;  /* us */
        double aerr = err < 0 ? -err : err;
        if (dt < c->min_dt)        c->min_dt = dt;
        if (dt > c->max_dt)        c->max_dt = dt;
        if (aerr > c->max_err_us)  c->max_err_us = aerr;
        c->sum_err_us += aerr;
    }
    c->done = 1;
}

int main(int argc, char** argv)
{
    g_ctx.cycles = argc > 1 ? atol(argv[1])          : 5000;
    g_ctx.period = argc > 2 ? (RTTIME)atoll(argv[2]) : 1000000ULL;
    int prio     = argc > 3 ? atoi(argv[3])          : 80;

    mlockall(MCL_CURRENT | MCL_FUTURE);
    init_lowlevel_logger(argc > 4 ? TRUE : FALSE);

    /* 2 MB stack, mirroring CRobot::InitRTTasks — also dodges the RT-POSIX
     * aarch64 bug where DEFAULT_STKSIZE(64K) < PTHREAD_STACK_MIN(128K) -> EINVAL */
    const INT STK = 2 * 1024 * 1024;

    g_ctx.rt_mode = 1;
    if (create_rt_task(&g_task, (const PCHAR)"GATE0", STK, prio) != RET_SUCC ||
        set_task_period(&g_task, SET_TM_NOW, g_ctx.period)       != RET_SUCC ||
        start_task(&g_task, &periodic_proc, &g_ctx)              != RET_SUCC) {
        printf("[GATE0] RT mode unavailable (rtprio/EPERM?) -> NRT fallback (mechanics only)\n");
        memset(&g_task, 0, sizeof(g_task));
        g_ctx.rt_mode = 0;
        if (create_nrt_task(&g_task, (const PCHAR)"GATE0N", STK) != RET_SUCC ||
            set_task_period(&g_task, SET_TM_NOW, g_ctx.period)   != RET_SUCC ||
            start_task(&g_task, &periodic_proc, &g_ctx)          != RET_SUCC) {
            printf("[GATE0] FAIL: cannot start a periodic task in any mode\n");
            return 1;
        }
    }

    while (!g_ctx.done)
        usleep(10000);

    double avg = g_ctx.sum_err_us / (double)g_ctx.cycles;
    printf("[GATE0] mode=%s cycles=%ld period=%llu ns\n",
           g_ctx.rt_mode ? "RT(SCHED_FIFO)" : "NRT(mechanics)",
           g_ctx.cycles, (unsigned long long)g_ctx.period);
    printf("[GATE0] dt min/max = %llu / %llu ns | |err| avg/max = %.1f / %.1f us\n",
           (unsigned long long)g_ctx.min_dt, (unsigned long long)g_ctx.max_dt,
           avg, g_ctx.max_err_us);
    printf("[GATE0] %s\n",
           g_ctx.rt_mode ? (g_ctx.max_err_us < 500.0 ? "PASS" : "CHECK (max err >= 500us)")
                         : "NRT-DONE (run with sudo or after re-login for RT numbers)");
    return 0;
}
