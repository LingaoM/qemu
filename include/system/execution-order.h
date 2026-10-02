/*
 * Deterministic execution ordering between QEMU threads
 *
 * Copyright 2026 Xiaomi Corporation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef SYSTEM_EXECUTION_ORDER_H
#define SYSTEM_EXECUTION_ORDER_H

void execution_order_init(void);
void execution_order_lock(void);
void execution_order_unlock(void);
bool execution_order_locked(void);

#endif /* SYSTEM_EXECUTION_ORDER_H */
