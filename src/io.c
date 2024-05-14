// Copyright 2024 Anti-Sway Team (Nguyen, Tri; Espinola, Malachi;
// Tevy, Vattanary; Hokenstad, Ethan; Neff, Callen)

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <pthread.h>
#include <math.h>

#include "MyRio.h"
#include "DIO.h"
#include "T1.h"
#include "conC_Encoder_initialize.h"

#include "error.h"
#include "thread-lib.h"

#include "io.h"

/* Reset Variable */
static bool reset;


/* Connector ID Convention */

// X Motor Encoder Connector ID (on Connector C)
#define X_CONNECTOR_ID 0
// Y Motor Encoder Connector ID (on Connector C)
#define Y_CONNECTOR_ID 1


/* Potentiometers */

// Best-Fit Potentiometer Slope (rad/V)
// TODO(nguy8tri): Find this quantity
#define POTENTIOMETER_SLOPE -2.11 * PI / 180.0
// Calibrated Voltage Intercept (x-intercept)
// for X Potentiometer
static float potentiometer_v_x_intercept;
// Calibrated Voltage Intercept (x-intercept)
// for Y Potentiometer
static float potentiometer_v_y_intercept;
// X Potentiometer
static MyRio_Aio x_potentiometer;
// Y Potentiometer
static MyRio_Aio y_potentiometer;


/* Encoders and Encoder Constants */


// X Motor Encoder
static MyRio_Encoder x_encoder;
// Y Motor Encoder
static MyRio_Encoder y_encoder;
// First Encoder state for both the
// X and Y Encoders
static int32_t first_enc_state[2];
// Previous Encoder state (from the last time
// either GetTrolleyPosition or GetTrolleyVelocity
// is caled), for both the X and Y Encoders
static int32_t prev_enc_state[2];
// Indicator if the holding for velocity is set
static bool holding_vel_set;
// Indicator if the holding for position is set
static bool holding_pos_set;
// Encoder Holding for velocity
static Velocities holding_vel;
// Encoder Holding for position
static Positions holding_pos;

// Number of counts in one revolution
// TODO(nguy8tri): Find this quantity
#define ENC_CNT_REV 2000.0
// Meters per revolution
// Diameter of upper pulley (12 mm) * PI
#define M_PER_REV 0.01267 * PI
/**
 * Converts a BDI quantity to meters
 * 
 * @param value THe BDI to convert
*/
#define ENC_2_POS(value) \
    (value) / ENC_CNT_REV * M_PER_REV
/**
 * Converts a BDI/BTI quantity to meters per second
 * 
 * @param The value to convert
*/
#define ENC_2_VEL(value) \
    (value) / (BTI_S * ENC_CNT_REV) * M_PER_REV



/* Motors and Motor Constants */


// X Motor Voltage Channel
MyRio_Aio x_motor;
// Y Motor Voltage Channel
MyRio_Aio y_motor;


/* Reference Velocity Definitions */


// The unit velocity stop corresponding
// to a keypad touch (m/s)
#define UNIT_VEL 0.1

/**
 * Holds booleans indicating which
 * buttons (1 through 9) are being pressed
*/
typedef bool Keymap[9];

/* Keypad Definitions and Variables */
// Number of Channels
#define CHANNELS 16
// Keypad Length
#define LCD_KEYPAD_LEN 4
// Keyboard channels
static MyRio_Dio channel[CHANNELS];
// Keyboard lock
static pthread_mutex_t keyboard;
// Our keymap
static Keymap keymap;
// Thread for Keymap Thread
static pthread_t keymap_thread;
// Thread Resource for Keymap Thread
static ThreadResource keymap_resource;

/* XY (Encoder) Bounds */

// Lower X Limit
#define X_LIM_LO 0
// Lower Y Limit
#define Y_LIM_LO 0
// Higher X Limit
#define X_LIM_HI 0.3
// Higher Y Limit
#define Y_LIM_HI 0.3
// Absolute Velocity Limit
#define VEL_LIM_ABS 0.5


/* Potentiometer Saturation Bounds */


// Lower Angle Saturation Limit
#define ANG_LIM_LO -15.0
// Upper Angle Saturation Limit
#define ANG_LIM_HI 15.0


/* Error Handling */
static int error;

MyRio_IrqTimer timer;


/* Static Helper Functions */


/**
 * Obtains the numerical buttons pressed
 * (1 through 9)
 * 
 * @param keymap The thread resource to signal
 * this thread when to stop
 * 
 * @return NULL
 * 
 * @post Updates keymap with all the number buttons,
 * excluding 0, that are pressed
*/
static inline void *KeymapThread(void *resource);

/**
 * Handles Error Processing from Position/Velocity
 * Measurements
 * 
 * @param curr_pos The current position
 * @param curr_vel The current velocity
 * 
 * @return 0 upon no error, negative otherwise (using the universal
 * error codes)
 * 
 * @post Iff negative is returned, both motors are switched off
*/
static inline int HandleEncoderError(Positions *curr_pos,
                                      Velocities *curr_vel);


/**
 * Handles Error Processing for Potentiometer Measurements
 * 
 * @param curr_ang The current angle reading
 * 
 * @return 0 upon no error, ESTRN otherwise
*/
static inline int HandlePotentiometerError(Angles *curr_ang);


/* Setup/Shutdown Functions */


int IOSetup() {
    // Setup Timer
    timer.timerWrite = IRQTIMERWRITE;
    timer.timerSet = IRQTIMERSETTIME;

    // Setup Encoders Channels
    conC_Encoder_initialize(myrio_session, &x_encoder, X_CONNECTOR_ID);
    conC_Encoder_initialize(myrio_session, &y_encoder, Y_CONNECTOR_ID);

    // Setup Potentiometer Voltage Channels (are swapped)
    Aio_InitCI1(&x_potentiometer);
    Aio_InitCI0(&y_potentiometer);

    // Calibration Message
    printf_lcd("\fPlease stablize for calibration.\n"
               "Press ENTR when ready.");
    while (getkey() != ENT) {}
    printf_lcd("\fCalibrating...\n");

    // Set Reference Positions
    first_enc_state[0] = Encoder_Counter(&x_encoder);
    first_enc_state[1] = Encoder_Counter(&y_encoder);

    // Setup the holding
    holding_vel_set = false;
    holding_pos_set = false;

    // Calibrate voltage intercepts for potentiometer
    potentiometer_v_x_intercept = Aio_Read(&x_potentiometer);
    potentiometer_v_y_intercept = Aio_Read(&y_potentiometer);

    // Setup Motor Channels
    Aio_InitCO0(&x_motor);
    Aio_InitCO1(&y_motor);

    // Setup Keyboard
    uint8_t i;
    for (i = 0; i < CHANNELS; i++) {
        channel[i].dir = DIOB_70DIR;
        channel[i].out = DIOB_70OUT;
        channel[i].in = DIOB_70IN;
        channel[i].bit = i;
    }
    VERIFY(error, pthread_mutex_init(&keyboard, NULL));

    printf_lcd("Calibration Finished\n");

    // Setup Reset flag
    reset = true;

    // Begin Keyboard Thread
    // START_THREAD(keymap_thread, KeymapThread, keymap_resource);

    return EXIT_SUCCESS;
}

int IOShutdown() {
    // Dissasociate with Encoders
    memset(&x_encoder, 0, sizeof(MyRio_Encoder));
    memset(&y_encoder, 0, sizeof(MyRio_Encoder));

    // Dissasociate with Potentiometers
    memset(&x_potentiometer, 0, sizeof(MyRio_Aio));
    memset(&y_potentiometer, 0, sizeof(MyRio_Aio));
    potentiometer_v_x_intercept = 0.0;
    potentiometer_v_y_intercept = 0.0;

    // Disassociate with Motor
    memset(&x_motor, 0, sizeof(MyRio_Aio));
    memset(&y_motor, 0, sizeof(MyRio_Aio));

    // Destroy Keymap Thread
    // STOP_THREAD(keymap_thread, keymap_resource);

    // Destroy Keyboard Lock
    VERIFY(error, pthread_mutex_destroy(&keyboard));

    return EXIT_SUCCESS;
}


/* Sensor Functions */

int GetReferenceVelocityCommand(Velocities *result) {
    // Setup discrete velocity comands,
    // -1, 0, and 1
    int8_t x_vel = 0;
    int8_t y_vel = 0;

    if (keymap[4]) {
        result->x_vel = 0;
        result->y_vel = 0;

        return EXIT_SUCCESS;
    }

    if (keymap[0] || keymap[1] || keymap[3]) y_vel++;
    if (keymap[2] || keymap[5] || keymap[8]) x_vel++;
    if (keymap[6] || keymap[7] || keymap[8]) y_vel--;
    if (keymap[0] || keymap[3] || keymap[6]) x_vel--;

    result->x_vel = x_vel * UNIT_VEL;
    result->y_vel = y_vel * UNIT_VEL;

    return EXIT_SUCCESS;
}

int GetReferenceAngleCommand(Angles *result) {
    result->x_angle = 0.0;
    result->y_angle = 0.0;

    return EXIT_SUCCESS;
}

int GetAngle(Angles *result) {
    float x_voltage = Aio_Read(&x_potentiometer);
    float y_voltage = Aio_Read(&y_potentiometer);

    result->x_angle =
        POTENTIOMETER_SLOPE * (x_voltage - potentiometer_v_x_intercept);
    result->y_angle =
        POTENTIOMETER_SLOPE * (y_voltage - potentiometer_v_y_intercept);

    return HandlePotentiometerError(result);
}

int GetTrolleyPosition(Positions *result) {
    static int32_t next_enc_state[2];
    if (reset) {
        prev_enc_state[0] = Encoder_Counter(&x_encoder);
        prev_enc_state[1] = Encoder_Counter(&y_encoder);
        reset = false;
        holding_vel_set = false;
        holding_pos_set = false;
    }

    if (holding_pos_set) {
        result->x_pos = holding_pos.x_pos;
        result->y_pos = holding_pos.y_pos;

        holding_pos_set = false;

        return EXIT_SUCCESS;
    }

    next_enc_state[0] = (int32_t) Encoder_Counter(&x_encoder);
    next_enc_state[1] = (int32_t) Encoder_Counter(&y_encoder);

    result->x_pos
        = ENC_2_POS((double) (next_enc_state[0] - first_enc_state[0]));
    result->y_pos
        = ENC_2_POS((double) (next_enc_state[1] - first_enc_state[1]));

    if (!holding_vel_set) {
        holding_vel.x_vel
            = ENC_2_VEL((double) (next_enc_state[0] - prev_enc_state[0]));
        holding_vel.y_vel
            = ENC_2_VEL((double) (next_enc_state[1] - prev_enc_state[1]));
        holding_vel_set = true;
    }

    prev_enc_state[0] = next_enc_state[0];
    prev_enc_state[1] = next_enc_state[1];

    return HandleEncoderError(result, &holding_vel);
}

int GetTrolleyVelocity(Velocities *result) {
    static int32_t next_enc_state[2];
    if (reset) {
        prev_enc_state[0] = Encoder_Counter(&x_encoder);
        prev_enc_state[1] = Encoder_Counter(&y_encoder);
        reset = false;
        holding_vel_set = false;
        holding_pos_set = false;

        result->x_vel = 0.0;
        result->y_vel = 0.0;

        return EXIT_SUCCESS;
    }

    if (holding_vel_set) {
        result->x_vel = holding_vel.x_vel;
        result->y_vel = holding_vel.y_vel;

        holding_vel_set = false;

        return EXIT_SUCCESS;
    }

    next_enc_state[0] = (int32_t) Encoder_Counter(&x_encoder);
    next_enc_state[1] = (int32_t) Encoder_Counter(&y_encoder);

    result->x_vel = ENC_2_VEL((double) (next_enc_state[0] - prev_enc_state[0]));
    result->y_vel = ENC_2_VEL((double) (next_enc_state[1] - prev_enc_state[1]));

    if (!holding_pos_set) {
        holding_pos.x_pos
            = ENC_2_POS((double) (next_enc_state[0] - first_enc_state[0]));
        holding_pos.y_pos
            = ENC_2_POS((double) (next_enc_state[1] - first_enc_state[1]));
        holding_pos_set = true;
    }

    prev_enc_state[0] = next_enc_state[0];
    prev_enc_state[1] = next_enc_state[1];

    return HandleEncoderError(&holding_pos, result);
}

int GetUserPosition(Angles *angle, Positions *pos, Positions *result) {
    result->x_pos = l * sin(angle->x_angle) + pos->x_pos;
    result->y_pos = l * sin(angle->y_angle) + pos->y_pos;

    return EXIT_SUCCESS;
}

int GetUserVelocity(Angles *angle, Velocities *vel, Velocities *result) {
    GetUserPosition(angle, (Positions *) vel, (Positions *) result);

    return EXIT_SUCCESS;
}


/* Actuator Functions */


int SetXVoltage(Voltage voltage) {
    Aio_Write(&x_motor, voltage);
    return EXIT_SUCCESS;
}

int SetYVoltage(Voltage voltage) {
    Aio_Write(&y_motor, -voltage);
    return EXIT_SUCCESS;
}


/* Keyboard Functions */


bool PressedDelete() {
#define DEL_ROW 7
#define DEL_COL 3
    pthread_mutex_lock(&keyboard);
    uint8_t j;
    for (j = 0; j < LCD_KEYPAD_LEN; j++) {
        Dio_WriteBit(channel + j, j == DEL_COL ? NiFpga_False : NiFpga_True);
    }

    if (!Dio_ReadBit(channel + DEL_ROW)) {
        pthread_mutex_unlock(&keyboard);
        return true;
    }

    pthread_mutex_unlock(&keyboard);
    return false;
#undef DEL_ROW
#undef DEL_COL
}


/* Static Helper Function */


static inline void *KeymapThread(void *resource) {
    ThreadResource *thread_resource = (ThreadResource *) resource;
    uint8_t i, j;

    while (thread_resource->irq_thread_rdy) {
        memset(keymap, false, sizeof(Keymap));
        pthread_mutex_lock(&keyboard);
        for (i = 0; i < LCD_KEYPAD_LEN - 1; i++) {
            for (j = 0; j < LCD_KEYPAD_LEN - 1; j++) {
                Dio_WriteBit(channel + j, i == j ? NiFpga_False : NiFpga_True);
            }
            for (j = LCD_KEYPAD_LEN; j < 2 * LCD_KEYPAD_LEN - 1; j++) {
                keymap[3 * (j - LCD_KEYPAD_LEN) + i]
                    = !Dio_ReadBit(channel + j);
            }
        }
        pthread_mutex_unlock(&keyboard);
    }

    EXIT_THREAD();
}


static inline int HandleEncoderError(Positions *curr_pos,
                                      Velocities *curr_vel) {
    // Check Positional Limits first
    u_error = EXIT_SUCCESS;
    if ((curr_pos->x_pos > X_LIM_HI && curr_vel->x_vel > 0) ||
        (curr_pos->x_pos < X_LIM_LO && curr_vel->x_vel < 0) ||
        (curr_pos->y_pos > Y_LIM_HI && curr_vel->y_vel > 0) ||
        (curr_pos->y_pos < Y_LIM_LO && curr_vel->y_vel < 0)) {
            u_error = EOTBD;
    }
    // Now, check velocity limits
    if (fabsf(curr_vel->x_vel) > VEL_LIM_ABS ||
        fabsf(curr_vel->y_vel) > VEL_LIM_ABS) {
        u_error = EVTYE;
    }
    // Output Error
    if (u_error) {
        SetXVoltage(0.0);
        SetYVoltage(0.0);
    }
    return u_error;
}

static inline int HandlePotentiometerError(Angles *curr_ang) {
    u_error = EXIT_SUCCESS;
    // if (curr_ang->x_angle < ANG_LIM_LO || curr_ang->x_angle > ANG_LIM_HI ||
    //     curr_ang->y_angle < ANG_LIM_HI || curr_ang->y_angle > ANG_LIM_HI) {
    //     u_error = ESTRN;
    // }
    // if (u_error) {
    //     SetXVoltage(0.0);
    //     SetYVoltage(0.0);
    // }
    return u_error;
}

/* Reset Functions */

void Reset() {
    reset = true;
}
