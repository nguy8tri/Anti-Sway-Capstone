/**
 * @file idle.c
 * @author Anti-Sway Team: Nguyen, Tri; Espinola, Malachi;
 * Tevy, Vattanary; Hokenstad, Ethan; Neff, Callen)
 * @brief Idle Mode Implementation
 * @version 0.1
 * @date 2024-06-03
 * 
 * @copyright Copyright (c) 2024
 * 
 */


#include <stdbool.h>
#include <pthread.h>
#include <stdint.h>
#include <stdlib.h>

#include "T1.h"

#include "setup.h"
#include "io.h"
#include "thread-lib.h"
#include "discrete-lib.h"


#include "idle.h"


/* Thread Number & Resources */


/// Thread ID
pthread_t idle_thread;
/// Thread Resources (Shared Resources)
ThreadResource resource;
/// Local Error Code
static int error;


/* Thread Function */


/**
 * @brief Idle Mode Thread Function
 * 
 * The Thread Function for Idle Mode
 * 
 * @param resource A pointer to a Resource sturcture
 * for Idle Mode
 * 
 * @return NULL
*/
static void *IdleModeThread(void *resource);


/* Tracking Mode Function Definitions */


int IdleFork() {
	printf("Starting Idle Mode\n");
    REGISTER_TIMER(resource);
    START_THREAD(idle_thread, IdleModeThread, resource);
    return EXIT_SUCCESS;
}

int IdleJoin() {
    STOP_THREAD(idle_thread, resource);
    UNREGISTER_TIMER(resource);
    return EXIT_SUCCESS;
}

static void *IdleModeThread(void *resource) {
	printf("Begin Idle Mode\n");
    ThreadResource *thread_resource = (ThreadResource *) resource;

    double t = 0.0;

    while (thread_resource->irq_thread_rdy) {
        uint32_t irq_assert = 0;
        TIMER_TRIGGER(irq_assert, thread_resource);
        static Positions trolley_pos;
        static Velocities trolley_vel;
        static Angles rope_ang;
        t += BTI_S;
        if (irq_assert) {
            // Do the loop for both motors

            // Get trolley info

            if (GetTrolleyPosition(&trolley_pos)) {
            	printf("Trolley Pos not okay\n");
            	EXIT_THREAD();
            }

            if (GetTrolleyVelocity(&trolley_vel)) {
            	printf("Trolley Vel not okay\n");
            	EXIT_THREAD();
            }

            if (GetAngle(&rope_ang)) {
            	EXIT_THREAD();
            }

            // Output the trolley info
/// How many decimal places to include
#define DECIMAL_PRECISION "3"
/// Radians to Degrees Conversion Factor
#define RAD_2_DEG(value) value * 180.0 / PI
            printf_lcd("\f"
                    "P:(%." DECIMAL_PRECISION "f, %."
                                            DECIMAL_PRECISION "f) m\n"
                    "V:(%." DECIMAL_PRECISION "f, %."
                                            DECIMAL_PRECISION "f) m/s"
                    "A:(%." DECIMAL_PRECISION "f, %."
                                    DECIMAL_PRECISION "f) deg\n\n",
                    trolley_pos.x_pos, trolley_pos.y_pos,
                    trolley_vel.x_vel, trolley_vel.y_vel,
                    RAD_2_DEG(rope_ang.x_angle),
                    RAD_2_DEG(rope_ang.y_angle));
#undef DECIMAL_PRECISION
#undef RAD_2_DEG

            Irq_Acknowledge(irq_assert);
        }
    }

    printf("Time: %f s", t);
    EXIT_THREAD();
}
