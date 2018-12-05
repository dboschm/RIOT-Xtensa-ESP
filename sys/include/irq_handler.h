/*
 * Copyright (C) 2018 Gunar Schorcht
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    sys_irq_handler Interrupt handler thread
 * @ingroup     sys
 * @brief       Single handler thread for interrupts in modules with
 *              blocking functions
 *
 * @author      Gunar Schorcht <gunar@schorcht.net>
 *
 * ## Interrupt Context Problem
 *
 * There are many modules that use shared peripheral interfaces such as SPI
 * and I2C, especially driver modules for sensors or actuators connected via
 * SPI or I2C. This type of modules also very often uses interrupts, which
 * normally require access to the SPI or the I2C interfaces when an interrupt
 * occurs, for example, to read status registers.
 *
 * Access to SPI and I2C interfaces is synchronized by mutual exclusion using
 * mutexes. If one thread tries to access such an interface that is already
 * being used by another thread, it will be blocked until the interface
 * becomes available. Although this synchronization works in the thread
 * context, it does not work in the interrupt context. Accessing such an
 * interface within an interrupt service routine would interfere with an
 * already existing interface access.
 *
 * The only solution to this problem is *not to call any function that
 * interacts with a device directly from interrupt context*. Rather, the
 * interrupt service routine only has to indicate the occurrence of the
 * interrupt. The interrupt is then handled asynchronously in a thread.
 *
 * The problem is that many modules, such as driver modules, do not use their
 * own thread, but run in the context of the calling thread. However, it can
 * not be up to the application thread to handle the interrupts of such
 * modules. The only solution would be for any module that uses interrupts
 * with SPI or I2C interfaces to create its own interrupt handler thread.
 * However, as the number of such modules increases, many resources (the
 * thread context including the thread stack) are used only for interrupt
 * handling.
 *
 * ## Solution
 *
 * The solution is to have only one thread with high priority for handling
 * the interrupts of such modules. For this purpose, the module defines a
 * prioritized interrupt event object for each interrupt source. The interrupt
 * event object contains also a reference to the function by which it has to
 * be handled.
 *
 * When an interrupt of the corresponding source occurs, the using module
 * registers the interrupt event object associated with the interrupt source
 * with the #irq_event_add function on the handler. The handler places the
 * interrupt event object in a pending interrupt queue according to its
 * priority. Each interrupt event object can be registered on the handler
 * only once. That is, if the same interrupt occurs multiple times, only its
 * first occurence is placed to the pending interrupt queue and is handled.
 *
 * When the interrupt handler thread gets the CPU, it processes all pending
 * interrupt events in the order of their occurence before it yields.
 *
 * ## Usage
 *
 * This interrupt handler thread can be used not only for blocking functions
 * such as SPI or I2C access, but also for other blocking functions. Please
 * note, however, that the handling of interrupts is serialized by this
 * module.
 *
 * To use the interrupt handler, the using module has to define a static
 * interrupt event object of type #irq_event_t for each interrupt source. The
 * interrupt event objects have to be initialized with the static initializer
 * #IRQ_EVENT_INIT. Furthermore, the interrupt handling function and optionally
 * an argument have to be set before it can be used.
 *
 * Once the interrupt event objects have been initialized, they can be added
 * with function #irq_event_add to the pending interrupt queue by an interrupt
 * service routine to indicate that an interrupt occured that has to be handled.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~ {.c}
 *      #include "foo_device.h"
 *      #include "irq_handler.h"
 *
 *      // interrupt event object with static initializer
 *      static irq_event_t _int_event = IRQ_EVENT_INIT;
 *
 *      ...
 *
 *      // no blocking interrupt service routine just adds the event and returns
 *      static void _int_request(void *arg)
 *      {
 *          irq_event_add(&_int_event, IRQ_EVENT_PRIO_HIGH);
 *      }
 *
 *      // example handler for the interrupt including blocking functions
 *      static void _int_handler(void *ctx)
 *      {
 *          foo_device_t dev = (foo_device_t*)ctx;
 *          uint8_t status;
 *
 *          // blocking access to the I2C
 *          i2c_aquire(dev->i2c_device);
 *          i2c_read_reg(dev->i2c_device, FOO_DEVICE_REG_STATUS, &status, 1);
 *          i2c_release(dev->i2c_device);
 *
 *          // application thread callbacks
 *          switch (status) {
 *              case FOO_INT_TYPE1:  dev->int_type1.isr(dev->int_type1.arg);
 *                                   break;
 *              case FOO_INT_TYPE2:  dev->int_type2.isr(dev->int_type2.arg);
 *                                   break;
 *              ...
 *          }
 *          ...
 *      }
 *
 *      ...
 *
 *      int foo_device_init(foo_device_t *dev, foo_device_params_t *params)
 *      {
 *          // set the handler for the interrupt
 *          _int_event.isr = _int_handler;
 *          _int_event.ctx = (void*)dev;
 *
 *          // initialize the GPIO wich interrupt service routine
 *          gpio_init_int(GPIO_PIN(0, 1), GPIO_IN, GPIO_BOTH, int_request, 0);
 *          ...
 *      }
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * @{
 * @file
 */

#ifndef IRQ_HANDLER_H
#define IRQ_HANDLER_H

#include <stdbool.h>

#include "assert.h"
#include "priority_queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Default priority of the interrupt handler thread
 *
 * The priority of the interrupt handler thread has to be high enough that all
 * pending interrupts are handled before other threads are executed.
 */
#ifndef IRQ_HANDLER_PRIO
#define IRQ_HANDLER_PRIO    0
#endif

/**
 * @brief   Interrupt event priorities
 *
 * Interrupt events are ordered in the pending interrupt queue
 * according to their priorities. Interrupts request of same priority are
 * handled in the order of their occurence.
 *
 * This type defines a very simple priority scheme with only three priority
 * levels which is sufficient in most cases: high, medium, low. Since the
 * priority is defined as integer value, the using modules might also use
 * priorities with finer granularity by using values between these predefined
 * values.
 */
typedef enum {
    IRQ_EVENT_PRIO_HIGH = 50,
    IRQ_EVENT_PRIO_MEDIUM = 100,
    IRQ_EVENT_PRIO_LOW = 150,
} irq_event_priority_t;

/**
 * @brief   Interrupt service routine prototype
 *
 * This type defines the prototype of the function that is registered together
 * with an interrupt event to be called when the interrupt is handled.
 */
typedef void (*irq_isr_t)(void *ctx);

/**
 * @brief   Interrupt event type
 *
 * The using module has to define an object of this type for each possible
 * interrupt source used by the module. Objects of this type are used to put
 * them in a pending interrupt queue indicating that an interrupt of the
 * corresponding source has occurred and needs to be handled. Each interrupt
 * event can only be pending once.
 *
 * Interrupt event objects have to be pre-allocated to use them.
 */
typedef struct {
    priority_queue_node_t node; /**< Priority queue node */
    bool pending;               /**< Indicates whether the same interrupt
                                     request event is already pending */
    irq_isr_t isr;              /**< Function to be called when the interrupt
                                     is handled */
    void *ctx;                  /**< Context used by the function */
} irq_event_t;

/**
 * @brief Static initializer for #irq_event_t.
 */
#define IRQ_EVENT_INIT { \
                            .node = PRIORITY_QUEUE_NODE_INIT, \
                            .pending = false, \
                            .isr = NULL, \
                            .ctx = NULL, \
                       }

/**
 * @brief   Initialize an interrupt event object.
 *
 * Initializes the given interrupt event object. Only use this function for
 * dynamically allocated interrupt event objects. For initialization of
 * static variables use #IRQ_EVENT_INIT instead.
 *
 * @param[out]  irq     Pre-allocated #irq_event_t object, must not be NULL
 */

static inline void irq_event_init(irq_event_t* irq)
{
    assert(irq != NULL);
    irq_event_t tmp = IRQ_EVENT_INIT;
    *irq = tmp;
}

/**
 * @brief   Add an prioritized interrupt event to the pending interrupt event
 *          queue
 *
 * The interrupt event given by parameter \p irq will be placed in pending
 * interrupt queue according to the priority given by the parameter \p prio.
 * It is appended in queue after pending interrupt events with the same
 * priority. Each interrupt event object can be added only once to the pending
 * intrrupt queue. That is, if the same interrupt occurs multiple times, only
 * its first occurence is placed to the pending interrupt queue and is handled.
 *
 * @param[in]   irq     Preallocated interrupt event object
 * @param[in]   prio    Priority of the interrupt event
 *
 * @retval 0        on success
 * @retval -EINVAL  if the given interrupt event is already pending
 */
int irq_event_add(irq_event_t* irq, irq_event_priority_t prio);

#ifdef __cplusplus
}
#endif

#endif /* IRQ_HANDLER_H */
/** @} */
