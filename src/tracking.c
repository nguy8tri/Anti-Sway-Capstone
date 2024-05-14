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
#include "record.h"

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
    Proportional combined_constants;
    Proportional damping;
} TrackingControlScheme;


/* Control-Loop Variables */

// Reference Angle (rad)
#define NOMINAL_REFERENCE_ANGLE 0.0
// The Control Scheme for the X Motor
static TrackingControlScheme x_control;
// The Control Scheme for the Y Motor
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
#define K_po -3000.0
// The artifical damping
#define B_t (8 * m_p / T_s)


/* Error Code */
static int error;


/* Data Recording Structures */


// The file ID
static FileID_t file = -1;
// The file Name
static char *data_file_name = "tracking.mat";
// The number of entries
#define DATA_LEN 10
// The data names
static char *data_names[DATA_LEN] = {"id", "t", "angle_x", "angle_y",
                                     "trolley_x", "trolley_y",
                                     "voltage_x", "voltage_y",
                                     "inner_x", "inner_y"};
// Buffer for data
static double data[DATA_LEN];
// Pointer to next data point to insert into buffer
static double *data_buff = data;
// ID variable
static int id = 1;

/**
 * Sets up a TrackingControl Scheme
 * 
 * @param scheme The scheme to setup
 * @param K_o The outer loop gain
 * @param K_i The inner loop gain
 * @param B The artificial damping to impose
 * 
 * @post scheme is setup with appropriate
 * outer/inner-loop control characteristics
*/
static inline void SetupScheme(TrackingControlScheme *scheme,
                               Proportional K_o,
                               Proportional K_i,
                               Proportional B);


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
 * @param pos_vel The measured velocity of the motor
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
                                     Velocity pos_vel,
                                     TrackingControlScheme *scheme,
                                     int (* SetVoltage)(Voltage voltage));


/* Tracking Mode Function Definitions */


int TrackingFork() {
    if (file == -1) {
        file = OpenDataFile(data_file_name, data_names, DATA_LEN);
    }
    SetupScheme(&x_control, K_po, K_pi, B_t);
    SetupScheme(&y_control, K_po, K_pi, B_t);
    REGISTER_TIMER(resource);
    START_THREAD(tracking_thread, TrackingModeThread, resource);
    return EXIT_SUCCESS;
}

int TrackingJoin() {
    STOP_THREAD(tracking_thread, resource);
    UNREGISTER_TIMER(resource);
    id++;
    return EXIT_SUCCESS;
}

static void *TrackingModeThread(void *resource) {
    ThreadResource *thread_resource = (ThreadResource *) resource;
    double t = 0.0;
    while (thread_resource->irq_thread_rdy) {
        static uint32_t irq_assert = 1;
        TIMER_TRIGGER(irq_assert, thread_resource);
        static Angles angle_ref;
        static Angles angle_input;
        static Velocities trolley_vel;

        if (irq_assert) {
            // Do the loop for both motors

            // Get the inputs
            if (GetReferenceAngleCommand(&angle_ref)) {
            	EXIT_THREAD();
            }
            if (GetAngle(&angle_input)) {
            	EXIT_THREAD();
            }
            if (GetTrolleyVelocity(&trolley_vel)) {
            	EXIT_THREAD();
            }
            // Record the sensor data
            *data_buff++ = id;
            *data_buff++ = t;
            *data_buff++ = angle_input.x_angle;
            *data_buff++ = angle_input.y_angle;
            *data_buff++ = trolley_vel.x_vel;
            *data_buff++ = trolley_vel.y_vel;

            // Run both control laws
            if (TrackingControlLaw(angle_ref.x_angle,
                                   angle_input.x_angle,
                                   trolley_vel.x_vel,
                                   &x_control,
                                   SetXVoltage)) {
            	EXIT_THREAD();
            }
            if (TrackingControlLaw(angle_ref.y_angle,
                                   angle_input.y_angle,
                                   trolley_vel.y_vel,
                                   &y_control,
                                   SetYVoltage)) {
            	EXIT_THREAD();
            }

            // Send data into file
            RecordData(file, data, DATA_LEN);
            data_buff = data;
            t += BTI_S;

            Irq_Acknowledge(irq_assert);
        }
    }
    printf("Time: %f s\n", t);
    EXIT_THREAD();
}

static inline int TrackingControlLaw(Angle angle_ref,
                                     Angle angle_input,
                                     Velocity pos_vel,
                                     TrackingControlScheme *scheme,
                                     int (* SetVoltage)(Voltage voltage)) {
    double outer_output = scheme->combined_constants *
        (angle_ref - angle_input);
    *data_buff++ = outer_output;
    double final_output = FORCE_TO_VOLTAGE(outer_output -
        scheme->damping * pos_vel);

    static int error;
    VERIFY(error, SetVoltage(final_output));
    *data_buff++ = final_output;

    return EXIT_SUCCESS;
}

static inline void SetupScheme(TrackingControlScheme *scheme,
                               Proportional K_o,
                               Proportional K_i,
                               Proportional B) {
    // K_po * K_pi / (K_pi + Bs) =
    //  AT * (1 + z^-1) / ((2B+CT)+(CT-2B)z^-1)
    // where A = K_poK_pi, B=B, C=K_pi
    scheme->combined_constants = K_o * K_i;
    scheme->damping = B;
}
