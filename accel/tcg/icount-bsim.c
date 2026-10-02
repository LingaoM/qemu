/*
 * BabbleSim integration for TCG instruction counting
 *
 * Copyright 2026 Xiaomi Corporation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"

#include "icount-bsim.h"

#include "bs_pc_base.h"

#include "exec/icount.h"
#include "qemu/error-report.h"
#include "qemu/main-loop.h"
#include "qemu/notify.h"
#include "qemu/seqlock.h"
#include "qemu/thread.h"
#include "qemu/timer.h"
#include "system/cpu-timers-internal.h"
#include "system/cpus.h"
#include "system/execution-order.h"
#include "system/runstate.h"
#include "system/system.h"

#define NS_PER_US 1000
#define DEFAULT_MRO_US 1000

typedef struct ICountBsimState {
    pb_dev_state_t device;
    Notifier exit_notifier;
    Notifier poll_notifier;
    int64_t now_ns;
    int64_t target_ns;
    int64_t committed_icount;
    int64_t target_icount;
    uint64_t mro_us;
    bool enabled;
    bool waiting;
    bool completion_pending;
    bool draining_io;
} ICountBsimState;

static ICountBsimState bsim_state;

static void icount_bsim_complete(ICountBsimState *state)
{
    seqlock_write_lock(&timers_state.vm_clock_seqlock,
                       &timers_state.vm_clock_lock);
    qatomic_set(&timers_state.qemu_icount_bias, state->target_ns);
    seqlock_write_unlock(&timers_state.vm_clock_seqlock,
                         &timers_state.vm_clock_lock);

    state->now_ns = state->target_ns;
    state->committed_icount = state->target_icount;
    state->completion_pending = true;
    qatomic_set(&state->waiting, false);

    qemu_clock_notify(QEMU_CLOCK_VIRTUAL);
}

static bool icount_bsim_poll_has_events(const GArray *pollfds)
{
    guint i;

    for (i = 0; i < pollfds->len; i++) {
        const GPollFD *pfd = &g_array_index(pollfds, GPollFD, i);

        if (pfd->revents != 0) {
            return true;
        }
    }

    return false;
}

static void icount_bsim_poll_notify(Notifier *notifier, void *opaque)
{
    ICountBsimState *state = container_of(notifier, ICountBsimState,
                                          poll_notifier);
    MainLoopPoll *poll = opaque;

    if (!state->draining_io) {
        return;
    }

    if (poll->state == MAIN_LOOP_POLL_FILL) {
        poll->timeout = 0;
    } else if (poll->state == MAIN_LOOP_POLL_OK &&
               !icount_bsim_poll_has_events(poll->pollfds)) {
        state->draining_io = false;
        icount_bsim_complete(state);
    }
}

static int64_t ns_to_bsim_us(int64_t ns)
{
    return DIV_ROUND_UP(ns, NS_PER_US);
}

static void icount_bsim_disconnect(Notifier *notifier, void *opaque)
{
    ICountBsimState *state = container_of(notifier, ICountBsimState,
                                          exit_notifier);

    if (!qatomic_read(&state->enabled)) {
        return;
    }
    main_loop_poll_remove_notifier(&state->poll_notifier);
    qemu_set_fd_handler(state->device.ff_ptd, NULL, NULL, NULL);
    pb_dev_disconnect(&state->device);
    qatomic_set(&state->enabled, false);
}

static void icount_bsim_response(void *opaque)
{
    ICountBsimState *state = opaque;
    int fd = state->device.ff_ptd;

    if (pb_dev_pick_wait_resp(&state->device) < 0) {
        qemu_set_fd_handler(fd, NULL, NULL, NULL);
        state->draining_io = false;
        qatomic_set(&state->waiting, false);
        qatomic_set(&state->enabled, false);
        error_report("BabbleSim PHY disconnected while advancing time");
        qemu_system_shutdown_request_with_code(SHUTDOWN_CAUSE_HOST_ERROR,
                                               EXIT_FAILURE);
        return;
    }

    /*
     * After acknowledging the wait, the PHY blocks waiting for this
     * device's next request.  Drain events already queued on the host
     * without blocking before allowing the vCPU to run again.
     */
    state->draining_io = true;
}

static void icount_bsim_request(int64_t target_ns, int64_t target_icount)
{
    pb_wait_t wait = { 0 };

    if (qatomic_read(&bsim_state.waiting)) {
        return;
    }

    target_ns = ns_to_bsim_us(target_ns) * NS_PER_US;
    if (target_ns <= bsim_state.now_ns) {
        return;
    }

    bsim_state.target_ns = target_ns;
    bsim_state.target_icount = target_icount;
    qatomic_set(&bsim_state.waiting, true);

    wait.end = target_ns / NS_PER_US;
    if (pb_dev_request_wait_nonblock(&bsim_state.device, &wait) < 0) {
        qatomic_set(&bsim_state.waiting, false);
        error_report("failed to request a BabbleSim time step");
        qemu_system_shutdown_request_with_code(SHUTDOWN_CAUSE_HOST_ERROR,
                                               EXIT_FAILURE);
    }
}

bool icount_bsim_configure(QemuOpts *opts, Error **errp)
{
    const char *sid = qemu_opt_get(opts, "bsim-sid");
    const char *phy = qemu_opt_get(opts, "bsim-phy");
    uint64_t device;

    if (!sid) {
        return true;
    }
    if (qemu_opt_get(opts, "rr")) {
        error_setg(errp, "BabbleSim and record/replay cannot be combined");
        return false;
    }

    device = qemu_opt_get_number(opts, "bsim-dev", UINT64_MAX);
    if (device > UINT_MAX) {
        error_setg(errp, "icount: bsim-dev must be specified");
        return false;
    }

    bsim_state.mro_us = qemu_opt_get_number(opts, "bsim-mro",
                                            DEFAULT_MRO_US);
    if (bsim_state.mro_us == 0 ||
        bsim_state.mro_us > INT64_MAX / NS_PER_US) {
        error_setg(errp, "icount: bsim-mro must be a positive number");
        return false;
    }

    if (pb_dev_init_com(&bsim_state.device, device, sid,
                        phy ? phy : "2G4") != 0) {
        error_setg(errp, "failed to connect icount to the BabbleSim PHY");
        return false;
    }

    /* Serialize main-loop I/O and vCPU execution. */
    execution_order_init();
    bsim_state.exit_notifier.notify = icount_bsim_disconnect;
    qemu_add_exit_notifier(&bsim_state.exit_notifier);
    bsim_state.poll_notifier.notify = icount_bsim_poll_notify;
    main_loop_poll_add_notifier(&bsim_state.poll_notifier);
    qemu_set_fd_handler(bsim_state.device.ff_ptd,
                        icount_bsim_response, NULL, &bsim_state);
    qatomic_set(&bsim_state.enabled, true);
    return true;
}

bool icount_bsim_enabled(void)
{
    return qatomic_read(&bsim_state.enabled);
}

bool icount_bsim_waiting(void)
{
    return qatomic_read(&bsim_state.waiting);
}

int64_t icount_bsim_limit_ns(int64_t deadline_ns)
{
    int64_t elapsed_icount;
    int64_t elapsed_ns;
    int64_t mro_ns = bsim_state.mro_us * NS_PER_US;
    int64_t limit_ns;

    limit_ns = deadline_ns < 0 ? mro_ns : MIN(deadline_ns, mro_ns);
    elapsed_icount = icount_get_raw() - bsim_state.committed_icount;
    elapsed_ns = icount_to_ns(MAX(elapsed_icount, 0));

    return MAX(limit_ns - elapsed_ns, 0);
}

bool icount_bsim_account_cpu(void)
{
    int64_t current_icount = icount_get_raw();
    int64_t elapsed_icount = current_icount - bsim_state.committed_icount;
    int64_t deadline_ns;
    int64_t elapsed_ns;

    if (icount_bsim_waiting()) {
        return false;
    }

    deadline_ns = qemu_clock_deadline_ns_all(QEMU_CLOCK_VIRTUAL,
                                             QEMU_TIMER_ATTR_ALL);
    if (icount_bsim_limit_ns(deadline_ns) > 0 &&
        !all_cpu_threads_idle()) {
        return true;
    }
    if (elapsed_icount <= 0) {
        return false;
    }

    elapsed_ns = icount_to_ns(elapsed_icount);
    icount_bsim_request(bsim_state.now_ns + elapsed_ns, current_icount);
    return false;
}

void icount_bsim_start_idle_wait(void)
{
    int64_t deadline_ns;
    int64_t delta_ns;

    if (bsim_state.completion_pending) {
        bsim_state.completion_pending = false;
        return;
    }
    if (icount_bsim_waiting() || !all_cpu_threads_idle()) {
        return;
    }

    deadline_ns = qemu_clock_deadline_ns_all(QEMU_CLOCK_VIRTUAL,
                                             ~QEMU_TIMER_ATTR_EXTERNAL);
    delta_ns = icount_bsim_limit_ns(deadline_ns);
    if (delta_ns <= 0) {
        if (delta_ns == 0) {
            qemu_clock_notify(QEMU_CLOCK_VIRTUAL);
        }
        return;
    }

    icount_bsim_request(bsim_state.now_ns + delta_ns,
                        bsim_state.committed_icount);
}
