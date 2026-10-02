/*
 * Deterministic execution ordering between QEMU threads
 *
 * Copyright 2026 Xiaomi Corporation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"

#include "qemu/main-loop.h"
#include "qemu/thread.h"
#include "system/execution-order.h"

static QemuMutex lock;
static QemuCond lock_cond;
static unsigned long lock_head;
static unsigned long lock_tail;
static __thread bool locked;
static bool enabled;

void execution_order_init(void)
{
    g_assert(!enabled);
    qemu_mutex_init(&lock);
    qemu_cond_init(&lock_cond);

    /* Hold the first ticket while execution threads are started. */
    locked = true;
    ++lock_tail;
    enabled = true;
}

bool execution_order_locked(void)
{
    return locked;
}

/* The execution-order lock must be taken before the BQL. */
void execution_order_lock(void)
{
    unsigned long ticket;

    if (!enabled) {
        return;
    }

    g_assert(!bql_locked());
    g_assert(!execution_order_locked());
    qemu_mutex_lock(&lock);
    ticket = lock_tail++;
    while (ticket != lock_head) {
        qemu_cond_wait(&lock_cond, &lock);
    }
    locked = true;
    qemu_mutex_unlock(&lock);
}

void execution_order_unlock(void)
{
    if (!enabled) {
        return;
    }

    g_assert(execution_order_locked());
    qemu_mutex_lock(&lock);
    ++lock_head;
    locked = false;
    qemu_cond_broadcast(&lock_cond);
    qemu_mutex_unlock(&lock);
}
