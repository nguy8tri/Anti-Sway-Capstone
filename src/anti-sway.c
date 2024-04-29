// Copyright 2024 Anti-Sway Team (Nguyen, Tri; Espinola, Malachi;
// Tevy, Vattanary; Hokenstad, Ethan; Neff, Callen)

#include <stdbool.h>
#include <pthread.h>
#include <stdint.h>

#include "setup.h"
#include "io.h"
#include "thread-lib.h"
#include "discrete-lib.h"

#include "anti-sway.h"


/* Thread Number & Resources */


// Thread ID
pthread_t anti_sway_thread = NULL;
// Thread Resources (Shared Resources)
ThreadResource anti_sway_resource;


/* Inner-Outer Loop Control Definition */


// The proportional constant for inner-loop
#define K_p 37.7
// The integral constant for inner-loop control
#define K_i 25.1


/**
 * Represents the Inner and Outer Loop Elements
*/
typedef struct {
    // Outer feedback
    Proportional outer_feedback;
    Proportional inner_prop;
    Integrator inner_int;
} AntiSwayControlScheme;


/* Control-Loop Variables */


// The Control Scheme for the X Motor
// TODO(nguy8tri): Define this statically
static AntiSwayControlScheme x_control;
// The Control Scheme for the Y Motor
// TODO(nguy8tri): Define this statically
static AntiSwayControlScheme y_control;


/* Scheme Setup Functions */
/**
 * Sets up an AntiSwayControlScheme
 * 
 * @param scheme The scheme to setup
 * 
 * @post scheme is now setup with zero
 * initial conditions and proper constants
*/
static inline void SetupScheme(AntiSwayControlScheme *scheme);


/* Thread Functions */


/**
 * The Thread Function for Anti-Sway Mode
 * 
 * @param resource A pointer to a Resource sturcture
 * for Tracking Mode
 * 
 * @return NULL
*/
static void *AntiSwayModeThread(void *resource);

/**
 * Executes 1 timestep for the Anti-Sway Mode Control Law
 * for its input to the plant
 * 
 * @param vel_ref The reference velocity for Anti-Sway Mode
 * @param angle_input The measured rope angle for Anti-Sway Mode
 * @param vel_input The measured velocity of the motor
 * @param scheme A pointer to the AntiSwayControlScheme structure
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
static inline int AntiSwayControlLaw(Velocity vel_ref,
                                     Angle angle_input,
                                     Velocity vel_input,
                                     AntiSwayControlScheme *scheme,
                                     int (* SetVoltage)(Voltage voltage));


/* Anti-Sway Mode Function Definitions */


int AntiSwayFork() {
    SetupScheme(&x_control);
    SetupScheme(&y_control);
    REGISTER_TIMER(anti_sway_resource);
    START_THREAD(anti_sway_thread, AntiSwayModeThread, anti_sway_resource);
    return EXIT_SUCCESS;
}

int AntiSwayJoin() {
    STOP_THREAD(anti_sway_thread, anti_sway_resource);
    UNREGISTER_TIMER(anti_sway_resource);
    return EXIT_SUCCESS;
}

static void *AntiSwayModeThread(void *resource) {
    ThreadResource *thread_resource = (ThreadResource *) resource;

    while (thread_resource->irq_thread_rdy) {
        uint32_t irq_assert = 0;
        TIMER_TRIGGER(irq_assert, thread_resource);
        Velocities reference_vel;  // Reference Velocity
        Angles input;  // Rope Angle
        Velocity trolley_vel;  // Trolley Velocity

        if (irq_assert) {
            // Do the loop for both motors

            // Get the inputs
            if (GetReferenceVelocityCommand(&reference_vel)) EXIT_THREAD();
            if (GetAngle(&input)) EXIT_THREAD();
            if (GetTrolleyVelocity(&trolley_vel)) EXIT_THREAD();
            // Run both control laws
            if (AntiSwayControlLaw(reference_vel.x_vel,
                                   input.x_angle,
                                   trolley_vel.x_vel,
                                   &x_control,
                                   SetXVoltage)) EXIT_THREAD();
            if (AntiSwayControlLaw(reference_vel.y_vel,
                                   input.y_angle,
                                   trolley_vel.y_vel,
                                   &y_control,
                                   SetYVoltage)) EXIT_THREAD();
        }
    }

    EXIT_THREAD();
}

static inline int AntiSwayControlLaw(Velocity vel_ref,
                                     Angle angle_input,
                                     Velocity vel_input,
                                     AntiSwayControlScheme *scheme,
                                     int (* SetVoltage)(Voltage voltage)) {
    double outer_output = vel_ref + scheme->outer_feedback * angle_input;

    Voltage final_output = PID(FORCE_TO_VOLTAGE(outer_output - vel_input),
                               &(scheme->inner_prop),
                               &(scheme->inner_int),
                               NULL);

    static int error;
    VERIFY(error, SetVoltage(final_output));

    return EXIT_SUCCESS;
}

static inline void SetupScheme(AntiSwayControlScheme *scheme) {
    scheme->outer_feedback = l * g;
    scheme->inner_prop = (m_p * m_t) * K_p
    IntegratorInit(K_i, BTI_S, &(scheme->inner_int));
}
