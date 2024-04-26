// Copyright 2024 Tri Nguyen

#ifndef THREAD_LIB_H_
#define THREAD_LIB_H_

#include <stdbool.h>
#include <pthread.h>

#include "DIIRQ.h"
#include "TimerIRQ.h"
#include "io.h"

/* Thread Data Structures */

/**
 * Defines the general ThreadResource
*/
typedef struct {
    NiFpga_IrqContext irq_context;  // context
    NiFpga_Bool irq_thread_rdy;  // stop signal
} ThreadResource;


/* Global Constants */


// The timestep, in microseconds (us)
#define BTI_US 5000
// The timestep, in milliseconds (ms)
#define BTI_MS 5
// The timestep, in seconds (s)
#define BTI_S 0.005


/* Thread Construction/Destruction */

/**
 * Starts a thread
 * 
 * @param thread The pthread_t ID variable to hold the thread's ID
 * @param function The Thread Function to execute for the thread
 * @param resource The ThreadResource to give the function
 * 
 * @post thread will contain the new PID (Process ID) of the thread
 * @post A new thread that runs function will now be running concurrently
 * 
 * @return EXIT_FAILURE upon failure to initialize the thread
*/
#define START_THREAD(thread, function, resource) \
    resource.irq_thread_rdy = true; \
    if (pthread_create(&thread, NULL, function, &resource)) return EXIT_FAILURE

/**
 * Registers the timer (global) with a particular thread (via its resource)
 * 
 * @param resource The ThreadResource associated with a thread
 * 
 * @post The thread associated with resource is now associated with the global timer
*/
#define REGISTER_TIMER(resource) \
    Irq_RegisterTimerIrq(&timer, &(resource.irq_context), BTI_US)

/**
 * Signals a Thread using a ThreadResource object to stop
 * 
 * @param thread The pthread_t holding the ID of the thread
 * to stop
 * @param resource The ThreadResource associated with the thread
 * 
 * @return EXIT_FAUILURE upon failure
 * 
 * @pre The thread uses resource, and calls EXIT_THREAD() when
 * resource.irq_thread_rdy is set to false
 * @post The thread associated with pthread_t is now done
*/
#define STOP_THREAD(thread, resource) \
    resource.irq_thread_rdy = false; \
    if (pthread_join(thread, NULL)) return EXIT_FAILURE

/**
 * Dissasociates a thread with a timer (via its resource)
 * 
 * @param resource The ThreadResource to disassociate the global timer with
 * 
 * @post The thread associated with resource is now disassociated with timer
*/
#define UNREGISTER_TIMER(resource) \
    Irq_UnregisterTimerIrq(&timer, resource.irq_context);

/**
 * Waits for a timer trigger (at the appropriate time step)
 * 
 * @param irq_assert A uint32_t that shall hold the assertion code
 * @param resource A pointer to a ThreadResource for the thread associated
 * with the global timer
 * 
 * @post irq_assert will be non-zero iff the timer has waited for the standard
 * time step (BTI_S/MS/US)
 * @post The timer will trigger after waiting for the standard time step (BTI_S/MS/US)
*/
#define TIMER_TRIGGER(irq_assert, resource) \
    Irq_Wait(resource->irq_context,
             TIMERIRQNO,
             &irq_assert,
             (NiFpga_Bool *) &(resource->irq_thread_rdy)); \
    NiFpga_WriteU32(myrio_session, IRQTIMERWRITE, BTI_US); \
    NiFpga_WriteBool(myrio_session, IRQTIMERSETTIME, NiFpga_True)

/**
 * Kills a Thread
 * 
 * @post The thread associated with the function that calls this is
 * now gone.
*/
#define EXIT_THREAD() \
    pthread_exit(NULL); \
    return NULL

#endif  // THREAD_LIB_H_
