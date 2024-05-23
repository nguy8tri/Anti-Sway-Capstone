// Copyright 2024 Anti-Sway Team (Nguyen, Tri; Espinola, Malachi;
// Tevy, Vattanary; Hokenstad, Ethan; Neff, Callen)

#include <stdbool.h>
#include <pthread.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "setup.h"
#include "io.h"
#include "thread-lib.h"
#include "discrete-lib.h"
#include "record.h"

#include "anti-sway.h"


/* Thread Number & Resources */


// Thread ID
pthread_t anti_sway_thread = NULL;
// Thread Resources (Shared Resources)
ThreadResource anti_sway_resource;


/* Inner-Outer Loop Control Characteristics */


// The proportional constant for inner-loop
static double K_ptx = 54.109;
// The integral constant for inner-loop control
static double K_itx = 30.891;
// The proportional constant for inner-loop
static double K_pty = 52.706;
// The integral constant for inner-loop control
static double K_ity = 28.235;

/* Control Loop Scheme */


/**
 * Represents the Inner and Outer Loop Elements
*/
typedef struct {
    // Outer feedback
    Proportional outer_feedback;
    // Inner PI Control
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


/* Error Code */


static int error;


/* Data Recording Structures */


// The file ID
static FileID_t file = -1;
// The file Name
static char *data_file_name = "anti-sway.mat";
// The number of entries
#define DATA_LEN 20
// The data names
static char *data_names[DATA_LEN] = {"id", "t",
                                     "vel_ref_x", "vel_ref_y",
                                     "angle_x", "angle_y",
                                     "trolley_vel_x", "trolley_vel_y",
                                     "vel_err_x", "voltage_x", "int_out_x", "Kp_x'", "Ki_x'", "loss_x",
                                     "vel_err_y", "voltage_y", "int_out_y", "Kp_y'", "Ki_y'", "loss_y"};
// Buffer for data
static double data[DATA_LEN];
// Pointer to next data point to insert into buffer
static double *data_buff = data;
// ID variable
static int id = 1;
// timestamp
static double t = 0.0;


/* Gradient Descent Variables */
// #define TUNING
#ifdef TUNING
static double dKp[2];
static double dKi[2];

#define ZERO_GRAD() \
	dKp[0] = 0.0; \
	dKp[1] = 0.0; \
	dKi[0] = 0.0; \
	dKi[1] = 0.0;

#define LR 10
static double prev_int[2][550];
static int prev_int_i = 0;
static bool int_res_first_time = true;
static double prev_Kp[2];
static double prev_Ki[2];
#endif


/* Scheme Setup Functions */


/**
 * Sets up an AntiSwayControlScheme
 * 
 * @param scheme The scheme to setup
 * @param K_p The proportional gain
 * @param K_i The integral gain
 * @param m The combined masses
 * 
 * @post scheme is now setup with zero
 * initial conditions and proper constants
*/
static inline void SetupScheme(AntiSwayControlScheme *scheme,
                               Proportional K_p,
                               Proportional K_i,
                               Proportional m);


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
    SetupScheme(&x_control, K_ptx, K_itx, m_dt + m_p);
    SetupScheme(&y_control, K_pty, K_ity, m_st + m_p);
    if (file == -1) {
        file = OpenDataFile(data_file_name, data_names, DATA_LEN);

		RecordValue(file, "MKp_x", x_control.inner_prop);
		RecordValue(file, "MKi_x", x_control.inner_int.gain * 2 / BTI_S);
		RecordValue(file, "K_x", x_control.outer_feedback);
		RecordValue(file, "MKp_y", y_control.inner_prop);
		RecordValue(file, "MKi_y", y_control.inner_int.gain * 2 / BTI_S);
		RecordValue(file, "K_y", y_control.outer_feedback);
    }
#ifdef TUNING
    ZERO_GRAD();
#endif
    KeyboardControlFork();
    REGISTER_TIMER(anti_sway_resource);
    START_THREAD(anti_sway_thread, AntiSwayModeThread, anti_sway_resource);
    return EXIT_SUCCESS;
}

int AntiSwayJoin() {
    STOP_THREAD(anti_sway_thread, anti_sway_resource);
    UNREGISTER_TIMER(anti_sway_resource);
    KeyboardControlJoin();
    SetXVoltage(0.0);
    SetYVoltage(0.0);
    // Update default gains
#ifdef TUNING
	K_ptx += LR * dKp[0] / (m_dt + m_p);
	K_itx += LR * dKi[0]/ (m_dt + m_p);
	K_itx = K_itx < 0.0 ? 40.0 : K_itx;
	K_pty += LR *  dKp[1] / (m_st + m_p);
	K_ity += LR * dKi[1] / (m_st + m_p);
	K_ity = K_ity < 0.0 ? 40.0 : K_ity;
	printf("Gradients: %.3f, %.3f, %.3f, %.3f\n", dKp[0], dKi[0], dKp[0], dKp[1]);
	printf("New gains: %.3f, %.3f, %.3f, %.3f\n", K_ptx, K_itx, K_pty, K_ity);
    prev_int_i = 0;
    int_res_first_time = false;
    prev_Kp[0] = K_ptx;
    prev_Kp[1] = K_pty;
    prev_Ki[0] = K_itx;
    prev_Ki[1] = K_ity;
#endif
    id++;
    t = 0.0;
    return EXIT_SUCCESS;
}

static void *AntiSwayModeThread(void *resource) {
    ThreadResource *thread_resource = (ThreadResource *) resource;

    while (thread_resource->irq_thread_rdy) {
        uint32_t irq_assert = 0;
        TIMER_TRIGGER(irq_assert, thread_resource);
        Velocities reference_vel = {0.0, 0.0};  // Reference Velocity
        Angles input;  // Rope Angle
        Velocities trolley_vel;  // Trolley Velocity

        if (irq_assert) {
            // Do the loop for both motors
            // Get the inputs
#ifdef TUNING
            reference_vel.x_vel = 0.15;
            reference_vel.y_vel= 0.15;
#else
            if (GetReferenceVelocityCommand(&reference_vel)) {
                EXIT_THREAD();
            }
#endif
            if (GetAngle(&input)) {
                EXIT_THREAD();
            }
            if (GetTrolleyVelocity(&trolley_vel)) {
                EXIT_THREAD();
            }
            // Record Data
            *data_buff++ = id;
            *data_buff++ = (t += BTI_S);
            *data_buff++ = reference_vel.x_vel;
            *data_buff++ = reference_vel.y_vel;
            *data_buff++ = input.x_angle;
            *data_buff++ = input.y_angle;
            *data_buff++ = trolley_vel.x_vel;
            *data_buff++ = trolley_vel.y_vel;
            // Run both control laws
            if (AntiSwayControlLaw(reference_vel.x_vel,
                                   input.x_angle,
                                   trolley_vel.x_vel,
                                   &x_control,
                                   SetXVoltage)) {
                EXIT_THREAD();
            }
            if (AntiSwayControlLaw(reference_vel.y_vel,
                                   input.y_angle,
                                   trolley_vel.y_vel,
                                   &y_control,
                                   SetYVoltage)) {
                EXIT_THREAD();
            }
            // Send data into file
            RecordData(file, data, DATA_LEN);
            data_buff = data;

            Irq_Acknowledge(irq_assert);
#ifdef TUNING
            if (++prev_int_i == 550) {
            	printf("Exiting Thread\n");
                EXIT_THREAD();

            }
#endif
        }
    }

    printf("Time: %f s\n", t);
    EXIT_THREAD();
}

static inline int AntiSwayControlLaw(Velocity vel_ref,
                                     Angle angle_input,
                                     Velocity vel_input,
                                     AntiSwayControlScheme *scheme,
                                     int (* SetVoltage)(Voltage voltage)) {
	static int i = 0;
    double outer_output = vel_ref + scheme->outer_feedback * angle_input;

    double vel_err =  outer_output - vel_input;
    *data_buff++ = vel_err;


    Voltage final_output = PID(FORCE_TO_VOLTAGE(vel_err),
                               &(scheme->inner_prop),
                               &(scheme->inner_int),
                               NULL,
                               MOTOR_V_LIM_L,
                               MOTOR_V_LIM_H);

    static int error;
    VERIFY(error, SetVoltage(final_output));
    *data_buff++ = final_output;
    *data_buff++ = scheme->inner_int.prev_output;

#ifdef TUNING
    static double prev_vel[2] = {0.0, 0.0};
	static double prev_V[2] = {0.0, 0.0};
    // "Back Propagation"
    double Ki = (scheme->inner_int.gain * 2 / BTI_S);
    double int_res = VOLTAGE_TO_FORCE(scheme->inner_int.prev_output) / Ki;

    double dIdKp = 0.0;
    double dIdKi = 0.0;
    if (!int_res_first_time) {
        dIdKp = (int_res - prev_int[i][prev_int_i]) / (scheme->inner_prop - prev_Kp[i]);
        dIdKi = (int_res - prev_int[i][prev_int_i]) / (Ki - prev_Ki[i]);
    }
    if (fabs(scheme->inner_prop - prev_Kp[i]) < 0.001) {
        dIdKp = 0.0;
    }
    if (fabs(Ki - prev_Ki[i]) < 0.001) {
        dIdKi = 0.0;
    }
    double dvdV =  (vel_input - prev_vel[i]) / (VOLTAGE_TO_FORCE(final_output - prev_V[i]));
    // We don't want NAN here
    if (fabs(final_output - prev_V[i]) < 0.001) {
    	dvdV = 0.0;
    }
	dKp[i] -= vel_err *  dvdV * (vel_err + Ki * dIdKp) / (1 + scheme->inner_prop * dvdV);
	// Use V_T_F here because the Integrator "thinks" its outputting a force, not a voltage
	dKi[i] -=  vel_err * dvdV * (int_res + Ki * dIdKi)  / (1 + scheme->inner_prop * dvdV);

	prev_vel[i] = vel_input;
	prev_V[i] = final_output;
    prev_int[i][prev_int_i] = int_res;
#endif

	*data_buff++ = scheme->inner_prop;
	*data_buff++ = scheme->inner_int.gain * 2 / BTI_S;
	*data_buff++ = vel_err * vel_err;

    if (++i == 2) i = 0;
    return EXIT_SUCCESS;
}

static inline void SetupScheme(AntiSwayControlScheme *scheme,
                               Proportional K_p,
                               Proportional K_i,
                               Proportional m) {
    scheme->outer_feedback = 2 * sqrt(l * g);
    scheme->inner_prop = m * K_p;
    IntegratorInit(K_i * m, BTI_S, &(scheme->inner_int));
}
