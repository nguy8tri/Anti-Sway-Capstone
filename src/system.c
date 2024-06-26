/**
 * @file system.c
 * @author Anti-Sway Team: Nguyen, Tri; Espinola, Malachi;
 * Tevy, Vattanary; Hokenstad, Ethan; Neff, Callen)
 * @brief System (Turing Machine)
 * @version 0.1
 * @date 2024-06-03
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <stdlib.h>
#include <stdbool.h>

#include "MyRio.h"
#include "T1.h"

#include "thread-lib.h"
#include "setup.h"
#include "anti-sway.h"
#include "tracking.h"
#include "idle.h"
#include "io.h"
#include "error.h"

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

/// State Functions
static int (* states[])() = {AntiSwayState,
                              TrackingState,
                              IdleState,
                              MenuState,
                              ErrorState,
                              StartState,
                              EndState};
/// Current State
static States state = START;


/* Error Handling */


/// Local Error Code
static int error;


/* Function Definitions */


int SystemExec() {
    VERIFY(error, StartState());

    state = MENU;

    while (state != END) {
        states[state]();
    }

    return states[state]();
}

static int AntiSwayState() {
    if (u_error) {
        AntiSwayJoin();
        state = ERROR;
    } else if (PressedDelete()) {
        AntiSwayJoin();
        state = MENU;
    }
    return EXIT_SUCCESS;
}

static int TrackingState() {
    if (u_error) {
        TrackingJoin();
        state = ERROR;
    } else if (PressedDelete()) {
        TrackingJoin();
        state = MENU;
    }
    return EXIT_SUCCESS;
}

static int IdleState() {
    if (u_error) {
        IdleJoin();
        state = ERROR;
    } else if (PressedDelete()) {
        IdleJoin();
        state = MENU;
    }
    return EXIT_SUCCESS;
}

static int MenuState() {
    printf_lcd("\f");
    printf_lcd("\n"
               "\t1) Tracking\n"
               "\t2) Anti-Sway\n"
               "\t3) Idle, 4) Exit\n"
               "Indicate a mode: ");

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
            IdleFork();
            state = IDLE;
            break;
        case '4':
            state = END;
            break;
        default:
            state = ERROR;
            return EXIT_FAILURE;
    }

    Reset();
    return EXIT_SUCCESS;
}

static int ErrorState() {
    SetXVoltage(0.0);
    SetYVoltage(0.0);
    if (u_error == ENKWN) {
        printf_lcd("\fAn unknown error has occurred. Exiting Program...\n");
        state = END;
        Shutdown();
        return EXIT_FAILURE;
    } else if (u_error == EOTBD || u_error == EVTYE || u_error == EENCR) {
        if (u_error == EOTBD) {
            printf_lcd("\fError: Positional Limit Exceeded");
        } else if (u_error == EVTYE) {
            printf_lcd("\fError: Velocity Limit Exceeded..");
        } else {
        	printf_lcd("\fError: An encoder(s) has failed..");
        }
        printf_lcd("Press:\n"
                   "1) Continue\n"
                   "2) Exit\n");
        int key;
        while ((key = getkey()) != '1' && key != '2') {}

        if (key == '2') {
            printf("\fExiting Program...\n");
            state = END;
            return EXIT_FAILURE;
        }
        state = MENU;
        u_error = 0;
    } else if (u_error == ESTRN) {
        printf_lcd("\fThe system has saturated unexpectedly."
                   "Exiting Program\n");
        state = END;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

static int StartState() {
    VERIFY(error, Setup());
    state = MENU;
    return EXIT_SUCCESS;
}

static int EndState() {
    VERIFY(error, Shutdown());
    return EXIT_SUCCESS;
}
