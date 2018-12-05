/*
 * Copyright (C) 2018 Gunar Schorcht
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

#include <inttypes.h>

#include "irq.h"
#include "irq_handler.h"
#include "thread.h"
#include "thread_flags.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

#define THREAD_FLAG_IRQ (1)

/* Stack for interrupt event handler thread */
static char _irq_handler_stack[THREAD_STACKSIZE_DEFAULT];

/* PID of interrupt handler thread, KERNEL_PID_UNDEF if not created yet */
static kernel_pid_t _irq_handler = KERNEL_PID_UNDEF;

/* Prioritized interrupt event queue */
static priority_queue_t irq_queue = PRIORITY_QUEUE_INIT;

static void *_irq_loop(void *arg)
{
    (void)arg;

    DEBUG("[%s] starts\n", __func__);

    while (1) {
        thread_flags_wait_any(THREAD_FLAG_IRQ);

        irq_event_t* irq;
        unsigned state = irq_disable();

        while ((irq = (irq_event_t*)priority_queue_remove_head(&irq_queue))) {
            /* make a local copy */
            DEBUG("[%s] handle irq %p, prio %"PRIu32"\n", __func__,
                   irq, irq->node.priority);
            irq_event_t irq_cpy = *irq;
            irq->pending = false;
            irq_restore(state);

            /* handle the pending interrupt */
            irq_cpy.isr(irq_cpy.ctx);

            state = irq_disable();
        }
        irq_restore(state);
    }
    return NULL;
}

int irq_event_add(irq_event_t* irq, irq_event_priority_t prio)
{
    assert(irq != NULL);
    assert(irq->isr != NULL);

    DEBUG("[%s] irq %p, prio %u\n", __func__, irq, prio);

    if (irq->pending) {
        DEBUG("[%s] interrupt event %p with prio %u is already pending\n",
              __func__, irq, prio);
        return -1;
    }

    /* disable interrupts */
    unsigned state = irq_disable();

    /* create the handler thread if not created yet */
    if (_irq_handler == KERNEL_PID_UNDEF) {
        DEBUG("[%s] create irq_handler thread\n", __func__);
        _irq_handler = thread_create(_irq_handler_stack,
                                     sizeof(_irq_handler_stack),
                                     IRQ_HANDLER_PRIO,
                                     THREAD_CREATE_WOUT_YIELD |
                                     THREAD_CREATE_STACKTEST,
                                     _irq_loop, NULL, "irq_handler");
        assert(_irq_handler != KERNEL_PID_UNDEF);
    }

    /* set the priority of the interrupt event and queue the it */
    irq->node.priority = prio;
    irq->pending = true;
    priority_queue_add(&irq_queue, (priority_queue_node_t*)irq);

    /* set the thread flag that interrupt events are pending */
    thread_flags_set((thread_t *)sched_threads[_irq_handler], THREAD_FLAG_IRQ);

    irq_restore(state);
    return 0;
}
