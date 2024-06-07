/**
 * @file tracking.c
 * @author Anti-Sway Team: Nguyen, Tri; Espinola, Malachi;
 * Tevy, Vattanary; Hokenstad, Ethan; Neff, Callen)
 * @brief Tracking Mode Control Law
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
 * @brief Tracking Mode Feedback Control Block
 * 
 * Represents the Inner and Outer Loop Elements
*/
typedef struct {
    //! Combined Outer-Loop Constant
    Proportional combined_constants;
    //! Artifical Damping (Inner Loop Feedback Gain)
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


// Local Error Flag
static int error;


/* Data Recording Structures */


// The file ID
static FileID_t file = -1;
// The file Name
static char *data_file_name = "tracking.mat";
// The number of entries
#define DATA_LEN 12
// The data names
static char *data_names[DATA_LEN] = {"id", "t",
                                     "angle_x", "angle_y",
                                     "trolley_pos_x", "trolley_pos_y",
                                     "trolley_vel_x", "trolley_vel_y",
                                     "inner_x", "voltage_x",
                                     "inner_y", "voltage_y"};
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
    SetupScheme(&x_control, -3295.3175, 1, 155.36);
    SetupScheme(&y_control, -1040.0, 1, 53.2);
    if (file == -1) {
        file = OpenDataFile(data_file_name, data_names, DATA_LEN);
        RecordValue(file, "K_x", x_control.combined_constants);
        RecordValue(file, "B_x", x_control.damping);
        RecordValue(file, "K_y", y_control.combined_constants);
        RecordValue(file, "B_y", y_control.damping);
    }

    REGISTER_TIMER(resource);
    START_THREAD(tracking_thread, TrackingModeThread, resource);
    return EXIT_SUCCESS;
}

int TrackingJoin() {
    STOP_THREAD(tracking_thread, resource);
    UNREGISTER_TIMER(resource);
    SetXVoltage(0.0);
    SetYVoltage(0.0);
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
        static Positions trolley_pos;
        static Velocities trolley_vel;

        if (irq_assert) {
            // Do the loop for both motors
            data_buff = data;

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
            if (GetTrolleyVelocity(&trolley_vel)) {
                EXIT_THREAD();
            }


            // Record the sensor data
            *data_buff++ = id;
            *data_buff++ = t;
            *data_buff++ = angle_input.x_angle;
            *data_buff++ = angle_input.y_angle;
            *data_buff++ = trolley_pos.x_pos;
            *data_buff++ = trolley_pos.y_pos;
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
