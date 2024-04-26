// Copyright 2024 Tri Nguyen

#ifndef IO_H_
#define IO_H_

#include "TimerIRQ.h"


/* Input/Output Data Types */

typedef float Angle;
typedef float Position;
typedef float Voltage;


/**
 * Defines the angle of the harness
 * along both directions, in radians
*/
typedef struct {
    Angle x_angle;
    Angle y_angle;
} Angles;

/**
 * Defines the position of an object
 * in 2D space, in meters
*/
typedef struct {
    Position x_pos;
    Position y_pos;
} Positions;

/* Sensor Variables */
// The Timer
MyRio_IrqTimer timer;

/* Actuator Limits */
// Motor Voltage High Limit (V)
#define MOTOR_V_LIM_H 10.000
// Motor Voltage Low Limit (V)
#define MOTOR_V_LIM_L -10.000

/* Physical Parameters */
// Pulley Radius (m)
#define R 0.003
// Current Constant (A/V)
#define K_a 0.41
// Motor Constant (Nm/A)
#define K_m 0.11


/* Setup/Shutdown Functions */


/**
 * Sets up the System-Sensor/Actuator
 * Interface
 * 
 * @return 0 upon success, negative
 * otherwise
*/
int IOSetup();

/**
 * Shutsdown the System-Sensor/Actuator
 * Interface
 * 
 * @return 0 upon success, negative
 * otherwise
*/
int IOShutdown();


/* Sensor Functions */

/**
 * Obtains the user command (for anti-sway)
 * 
 * @param result A return parameter, which
 * will become the change in position requested
 * by the user
 * 
 * @return 0 upon success, other integers
 * if otherwise
 * @return A position structure, which reflects
 * the change in position requested from the user
*/
int GetUserCommand(Positions *result);

/**
 * Obtains the angle of the harness
 * 
 * @param result A return parameter, which
 * will become the angle along both directions
 * 
 * @return 0 upon success, other integers
 * if otherwise
 * @return result, which will define the 
 * angle of the harness along 
 * both lateral directions
*/
int GetAngle(Angles *result);

/**
 * Obtains the Trolley Position
 * 
 * @param result A return parameter, which
 * will become the position of the motor
 * 
 * @return 0 upon success, other integers
 * if otherwise
 * @return A Position structure, which
 * defines the Position of the Motor in
 * the lateral plane
*/
int GetTrolleyPosition(Position *result);

/**
 * Obtains the User Position
 * 
 * @param result A return parameter, which
 * will become the position of the user
 * 
 * @return 0 upon success, other integers
 * if otherwise
 * @return A Position structure, which
 * defines the Position of the User in
 * the lateral plane
*/
int GetUserPosition(Position *result);


/* Actuator Functions */


/**
 * Sets the voltage of the X motor
 * 
 * @return 0 upon success, other integers
 * if otherwise
*/
int SetXVoltage(Voltage voltage);

/**
 * Sets the voltage of the Y motor
 * 
 * @return 0 upon success, other integers
 * if otherwise
*/
int SetYVoltage(Voltage voltage);

#endif  // IO_H_
