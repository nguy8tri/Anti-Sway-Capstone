// Copyright 2024 Anti-Sway Team (Nguyen, Tri; Espinola, Malachi;
// Tevy, Vattanary; Hokenstad, Ethan; Neff, Callen)

#include <stdlib.h>

#include "setup.h"
#include "anti-sway.h"
#include "tracking.h"

#include "system.h"

/* State Functions */

// State Functions
/**
 * Executes the Anti-Sway State, which includes
 * 1) Running Anti-Sway Mode
 * 2) Executing Transitions from this State
*/
static void AntiSwayState();

/**
 * Executes the Tracking Mode State, which includes
 * 1) Running Tracking Mode
 * 2) Executing Transitions from this State
*/
static void TrackingState();

/**
 * Executes the Idle State, which includes
 * 1) Doing Nothing
 * 2) Executing Transitions from this State
*/
static void IdleState();

/**
 * Executes the Error State, which includes
 * 1) Stopping the System
 * 2) Stopping any Concurrent Processes
 * 3) Deallocating all Resources
 * 4) Outputting the error
*/
static void ErrorState();

/**
 * Executes the Start State, which includes
 * 1) Setting up the System
 * 2) Executing the next state
 * Note: This is a once-only state
*/
static void StartState();

/**
 * Executes the End State, which includes
 * 1) Stopping the System
 * 2) Stopping all Concurrent Processes
 * 3) Deallocating all Resources
*/
static void EndState();

/* State Data Types */

/**
 * The possible states
*/
typedef enum {
    ANTI_SWAY,
    TRACKING,
    IDLE,
    ERROR,
    START,
    END
} States;

/* State Variables*/

// State Functions
static void (* states[])() = {AntiSwayState,
                              TrackingState,
                              IdleState,
                              ErrorState,
                              StartState,
                              EndState};
// Current State
static States state = START;


/* Function Definitions */


int SystemExec() {
    int error;
    VERIFY(error, Setup());

    while (state != ERROR && state != END) {
        states[state]();
    }

    if (state == ERROR) return states[state]();

    states[state]();

    VERIFY(error, Shutdown());

    return EXIT_SUCCESS
}

static void AntiSwayState() {
    AntiSwayFork();

    // TODO(nguy8tri): other code for tracking, error, and stop transitions
}

static void TrackingState() {
    TrackingFork();

    // TODO(nguy8tri): other code for anti-sway, error, and stop transitions
}

static void IdleState() {
    // TODO(nguy8tri): other code for tracking, anti-sway, and stop transitions
}

static void ErrorState() {
    // TODO(nguy8tri): Determine the error that happened and print it
}

static void StartState() {
    // TODO(nguy8tri): Print out the beginning message
}

static void EndState() {
    // TODO(nguy8tri): Print out a stopping message
}
