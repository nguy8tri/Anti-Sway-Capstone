// Copyright 2024 Anti-Sway Team (Nguyen, Tri; Espinola, Malachi;
// Tevy, Vattanary; Hokenstad, Ethan; Neff, Callen)

#include <stdlib.h>
#include <stdbool.h>

#include "MyRio.h"
#include "T1.h"

#include "thread-lib.h"
#include "setup.h"
#include "anti-sway.h"
#include "tracking.h"
#include "io.h"

#include "system.h"

/* State Functions */

// State Functions
/**
 * Executes the Anti-Sway State, which includes
 * 1) Running Anti-Sway Mode
 * 2) Executing Transitions from this State
 * 
 * @return 0 upon success, negative otherwise
*/
static int AntiSwayState();

/**
 * Executes the Tracking Mode State, which includes
 * 1) Running Tracking Mode
 * 2) Executing Transitions from this State
 * 
 * @return 0 upon success, negative otherwise
*/
static int TrackingState();

/**
 * Executes the Idle State, which includes
 * 1) Doing Nothing
 * 2) Executing Transitions from this State
 * 
 * @return 0 upon success, negative otherwise
*/
static int IdleState();

/**
 * Executes the Menu State, which includes
 * 1) Prompting for the next state
 * 2) Executing the next state
 * 
 * @return 0 upon success, negative otherwise
*/
static int MenuState();

/**
 * Executes the Error State, which includes
 * 1) Stopping the System
 * 2) Stopping any Concurrent Processes
 * 3) Deallocating all Resources
 * 4) Outputting the error
 * 
 * @return The error code from the failure
*/
static int ErrorState();

/**
 * Executes the Start State, which includes
 * 1) Setting up the System
 * 2) Executing the next state
 * Note: This is a once-only state
 * 
 * @return 0 upon success, negative otherwise
*/
static int StartState();

/**
 * Executes the End State, which includes
 * 1) Stopping the System
 * 2) Stopping all Concurrent Processes
 * 3) Deallocating all Resources
 * 
 * @return 0 upon success, negative otherwise
*/
static int EndState();

/* State Data Types */

/**
 * The possible states
*/
typedef enum {
    ANTI_SWAY,
    TRACKING,
    IDLE,
    MENU,
    ERROR,
    START,
    END
} States;

/* State Variables*/

// State Functions
static int (* states[])() = {AntiSwayState,
                              TrackingState,
                              IdleState,
                              MenuState,
                              ErrorState,
                              StartState,
                              EndState};
// Current State
static States state = START;


/* Error Handling */
static int error;


/* Function Definitions */


int SystemExec() {
    VERIFY(error, StartState());

    while (state != ERROR && state != END) {
        states[state]();
    }

    return states[state]();
}

static void AntiSwayState() {
    // TODO(nguy8tri): other code for tracking, error, and stop transitions
    if (PressedDelete()) {
        AntiSwayJoin();
        state = MENU;
    }
}

static void TrackingState() {
    // TODO(nguy8tri): other code for anti-sway, error, and stop transitions
    if (PressedDelete()) {
        TrackingJoin();
        state = MENU;
    }
}

static void IdleState() {
    // TODO(nguy8tri): other code for tracking, anti-sway, and stop transitions
    static Positions trolley_pos;
    static Velocities trolley_vel;
    static Angles rope_ang;
    if (GetTrolleyPosition(&trolley_pos) || GetTrolleyVelocity(&trolley_vel) ||
        GetAngle(&rope_ang)) {
        state = ERROR;
    }
#define DECIMAL_PRECISION 3
#define RAD_2_DEG(value) value * 180.0 / PI
    printf_lcd("\f"
               "Trolley Position: (%." DECIMAL_PRECISION "f, %."
                                       DECIMAL_PRECISION "f) m\n"
               "Trolley Velocity: (%." DECIMAL_PRECISION "f, %."
                                       DECIMAL_PRECISION "f) m/s\n"
                "Rope Angle: (%. " DECIMAL_PRECISION "f, %."
                                   DECIMAL_PRECISION "f) deg\n",
                trolley_pos->x_pos, trolley_pos->y_pos,
                trolley_vel->x_vel, trolley_pos->y_vel,
                RAD_2_DEG(rope_ang->x_angle), RAD_2_DEG(rope_ang->y_angle));
    if (PressedDelete()) {
        state = MENU;
    }
#undef DECIMAL_PRECISION
#undef RAD_2_DEG
}

static void MenuState() {
    printf_lcd("\fIndicate a mode to go to:\n"
               "\t1) Tracking\n"
               "\t2) Anti-Sway\n"
               "\t3) Idle, 4) Exit\n");

    int key;
    while (!('1' <= (key = getchar_keypad()) && key <= '4')) {}

    switch (key) {
        case '1':
            TrackingFork();
            state = TRACKING;
            break;
        case '2':
            AntiSwayFork();
            state = ANTI_SWAY;
            break;
        case '3':
            state = IDLE;
            break;
        case '4':
            state = END;
            break;
        default:
            state = ERROR;
            return EXIT_FAILURE;
    }

    reset = true;
    return EXIT_SUCCESS;
}

static void ErrorState() {
    // TODO(nguy8tri): Determine the error that happened and print it
    printf_lcd("An error has occured. Exiting Program\n");
    Shutdown();
    return EXIT_FAILURE;
}

static int StartState() {
    // TODO(nguy8tri): Print out the beginning message
    VERIFY(error, Setup());
    state = MENU;
    return EXIT_SUCCESS;
}

static int EndState() {
    // TODO(nguy8tri): Print out a stopping message
    VERIFY(error, Shutdown());
    return EXIT_SUCCESS;
}
