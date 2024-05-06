// Copyright 2024 Anti-Sway Team (Nguyen, Tri; Espinola, Malachi;
// Tevy, Vattanary; Hokenstad, Ethan; Neff, Callen)


#include <stdbool.h>
#include <pthread.h>
#include <stdint.h>
#include <stdlib.h>

#include "setup.h"
#include "io.h"
#include "thread-lib.h"
#include "discrete-lib.h"


#include "tracking.h"

/* Thread Number & Resources */


// Thread ID
pthread_t tracking_thread;
// Thread Resources (Shared Resources)
ThreadResource resource;


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

// Reference Angle (rad)
#define NOMINAL_REFERENCE_ANGLE 0.0
// The Control Scheme for the X Motor
// TODO(nguy8tri): Define this statically
static TrackingControlScheme x_control;
// The Control Scheme for the Y Motor
// TODO(nguy8tri): Define this statically
static TrackingControlScheme y_control;

// The settling time (0.1 s)
#define T_s 0.1
// The overshoot fraction (5%)
#define os 0.05
// TODO(nguy8tri): Make code that will
// automate this step
// The inner-loop proportional constant
#define K_pi 1.0
// The outer-loop proportional constant
#define K_po -100.43945510577155
// The artifical damping
#define B (8 * m_p / T_s)


/* Error Code */
static int error;

/**
 * Sets up a TrackingControl Scheme
 * 
 * @param scheme The scheme to setup
 * 
 * @post scheme is setup with appropriate
 * outer/inner-loop control characteristics
*/
static inline void SetupScheme(TrackingControlScheme *scheme);

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
 * @param angle_ref The reference angle for Tracking Mode
 * @param angle_input The measured rope angle for Tracking Mode
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
static inline int TrackingControlLaw(Angle angle_ref,
                                     Angle angle_input,
                                     Position pos_input,
                                     TrackingControlScheme *scheme,
                                     int (* SetVoltage)(Voltage voltage));


/* Tracking Mode Function Definitions */


int TrackingFork() {
    SetupScheme(&x_control);
    SetupScheme(&y_control);
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
    ThreadResource *thread_resource = (ThreadResource *) resource;

    while (thread_resource->irq_thread_rdy) {
        uint32_t irq_assert = 0;
        TIMER_TRIGGER(irq_assert, thread_resource);
        Angles angle_ref;
        Angles angle_input;
        Positions trolley_pos;

        if (irq_assert) {
            // Do the loop for both motors

            // Get the inputs
            if (GetReferenceAngleCommand(&angle_ref)) {
            	EXIT_THREAD();
            }
            if (GetAngle(&angle_input)) {
            	EXIT_THREAD();
            }
            if (GetTrolleyPosition(&trolley_pos)) {
            	EXIT_THREAD();
            }
            // Run both control laws
            if (TrackingControlLaw(angle_ref.x_angle,
                                   angle_input.x_angle,
                                   trolley_pos.x_pos,
                                   &x_control,
                                   SetXVoltage)) {
            	EXIT_THREAD();
            }
            if (TrackingControlLaw(angle_ref.y_angle,
                                   angle_input.y_angle,
                                   trolley_pos.y_pos,
                                   &y_control,
                                   SetYVoltage)) {
            	EXIT_THREAD();
            }
        }
    }

    EXIT_THREAD();
}

static inline int TrackingControlLaw(Angle angle_ref,
                                     Angle angle_input,
                                     Position pos_input,
                                     TrackingControlScheme *scheme,
                                     int (* SetVoltage)(Voltage voltage)) {
    double outer_output = Cascade(angle_ref - angle_input,
                                  &(scheme->outer_block),
                                  1,
                                  NEG_INF,
                                  POS_INF);

    Voltage final_output = PID(FORCE_TO_VOLTAGE(outer_output - pos_input),
                               &(scheme->inner_prop),
                               NULL,
                               &(scheme->inner_diff),
                               MOTOR_V_LIM_L,
                               MOTOR_V_LIM_H);

    static int error;
    VERIFY(error, SetVoltage(final_output));

    return EXIT_SUCCESS;
}

static inline void SetupScheme(TrackingControlScheme *scheme) {
    // K_po * K_pi / (K_pi + Bs) =
    //  K_po * K_pi * (1 + z^-1)/((K_pi+2*B/T)+(K_pi-2*B/T)*z^-1)
   Biquad new_b = {{K_po * K_pi, K_po * K_pi, 0.0},
                           {K_pi + 2 * B / BTI_S, K_pi - 2 * B / BTI_S, 0.0},
                           {0.0, 0.0},
                           {0.0, 0.0}};
    scheme->outer_block = new_b;
    scheme->inner_prop = K_pi;
    DifferentiatorInit(B, BTI_S, &(scheme->inner_diff));
}
