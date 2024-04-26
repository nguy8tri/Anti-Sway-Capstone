// Copyright 2024 Tri Nguyen

#include <stdbool.h>
#include <pthread.h>
#include <stdint.h>

#include "io.h"
#include "thread-lib.h"
#include "discrete-lib.h"

#include "tracking.h"


/* Thread Number & Resources */

// Thread ID
pthread_t tracking_thread = NULL;
// Thread Resources (Shared Resources)
Resource resource;


/* Inner-Outer Loop Control Definition */

/**
 * Represents the Inner and Outer Loop Elements
*/
typedef struct {
    Biquad outer_block;
    Proportional inner_prop;
    Differentiator inner_diff;
} TrackingControlScheme;


/* Control-Loop Variables */

// The Control Scheme for the X Motor
// TODO(nguy8tri): Define this statically
static TrackingControlScheme x_control;
// The Control Scheme for the Y Motor
// TODO(nguy8tri): Define this statically
static TrackingControlScheme y_control;


/* Thread Functions */

/**
 * The Thread Function for Tracking Mode
 * 
 * @param resource A pointer to a Resource sturcture
 * for Tracking Mode
 * 
 * @return NULL
*/
static void *TrackingModeThread(void *resource);

/**
 * Executes 1 timestep for the Tracking Mode Control Law
 * for its input to the plant
 * 
 * @param angle_input The angle input for Tracking Mode
 * @param pos_input The measured position of the motor
 * @param scheme A pointer to the TrackingControlScheme structure
 * used to execute the control law
 * @param SetVoltage The function that sets the voltage of the
 * appropriate motor
 * 
 * @return 0 upon success, negative otherwise
 * 
 * @pre scheme was not modified before use of this function
 * @post scheme is now updated with the input and outputs for
 * the respective control scheme
*/
static inline int TrackingControlLaw(Angle angle_input,
                                     Position pos_input,
                                     TrackingControlScheme *scheme,
                                     int (* SetVoltage)(Voltage voltage));

int TrackingFork() {
    REGISTER_TIMER(resource);
    START_THREAD(tracking_thread, TrackingModeThread, resource);
    return EXIT_SUCCESS;
}

int TrackingJoin() {
    STOP_THREAD(tracking_thread, resource);
    UNREGISTER_TIMER(resource);
    return EXIT_SUCCESS;
}

static void *TrackingModeThread(void *resource) {
    TrackingResource *thread_resource = (TrackingResource *) resource;

    while (thread_resource->irq_thread_rdy) {
        uint32_t irq_assert = 0;
        TIMER_TRIGGER(irq_assert, thread_resource);
        Angles input;
        Positions trolley_pos;

        if (irq_assert) {
            // Do the loop for both motors

            // Get the inputs
            if (GetAngle(&input)) EXIT_THREAD();
            if (GetTrolleyPosition(&trolley_pos)) EXIT_THREAD();
            // Run both control laws
            if (TrackingControlLaw(input.x_angle,
                                   trolley_pos.x_pos,
                                   &x_control,
                                   SetXVoltage)) EXIT_THREAD();
            if (TrackingControlLaw(input.y_angle,
                                   trolley_pos.y_pos,
                                   &y_control,
                                   SetYVoltage)) EXIT_THREAD();
        }
    }

    EXIT_THREAD();
}

static inline int TrackingControlLaw(Angle angle_input,
                                     Position pos_input,
                                     TrackingControlScheme *scheme,
                                     int (* SetVoltage)(Voltage voltage)) {
    double outer_output = Cascade((double) input,
                                  &(scheme->outer_block),
                                  1,
                                  NEG_INF,
                                  POS_INF);

    Voltage final_output = PID((outer_output - pos_input) * R / (K_m * K_a),
                               &(scheme->inner_prop),
                               NULL,
                               &(scheme->inner_diff));

    if (SetVoltage(final_output)) return EXIT_FAILURE;

    return EXIT_SUCCESS;
}
