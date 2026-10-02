/*
 * BabbleSim integration for TCG instruction counting
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef TCG_ICOUNT_BSIM_H
#define TCG_ICOUNT_BSIM_H

#include "qapi/error.h"
#include "qemu/option.h"

#ifdef CONFIG_BSIM

bool icount_bsim_configure(QemuOpts *opts, Error **errp);
bool icount_bsim_enabled(void);
bool icount_bsim_waiting(void);
int64_t icount_bsim_limit_ns(int64_t deadline_ns);
bool icount_bsim_account_cpu(void);
void icount_bsim_start_idle_wait(void);

#else

static inline bool icount_bsim_configure(QemuOpts *opts, Error **errp)
{
    if (qemu_opt_get(opts, "bsim-sid")) {
        error_setg(errp, "QEMU was built without BabbleSim support");
        return false;
    }
    return true;
}

static inline bool icount_bsim_enabled(void)
{
    return false;
}

static inline bool icount_bsim_waiting(void)
{
    return false;
}

static inline int64_t icount_bsim_limit_ns(int64_t deadline_ns)
{
    return deadline_ns;
}

static inline bool icount_bsim_account_cpu(void)
{
    return false;
}

static inline void icount_bsim_start_idle_wait(void)
{
}

#endif

#endif /* TCG_ICOUNT_BSIM_H */
