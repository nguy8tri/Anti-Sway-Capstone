// Copyright 2024 Anti-Sway Team (Nguyen, Tri; Espinola, Malachi;
// Tevy, Vattanary; Hokenstad, Ethan; Neff, Callen)

#ifndef IO_H_
#define IO_H_

#include <stdbool.h>

#include "TimerIRQ.h"


/* Reset Feature */
// Reset Flag for GetTrolleyPosition
// and GetVelocityPosition
bool reset = true;


/* Input/Output Data Types */

typedef float Angle;
typedef float Position;
typedef float Velocity;
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

/**
 * Defines the velocity of an object
 * in 2D space, in meters/second
*/
typedef struct {
    Velocity x_vel;
    Velocity y_vel;
} Velocities;

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
/**
 * Force to Voltage Conversion
 * 
 * @param force An int/float/double
 * expression, which represents the force
 * to transmit
 * 
 * @post Becomes the conversion between
 * force to the voltage to output
*/
#define FORCE_TO_VOLTAGE(force) \
    (force) * R / (K_a * K_m)


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
 * @return 0 upon success, negative otherwise
 * @return A Velocities structure, which reflects
 * the change in position requested from the user
*/
int GetReferenceVelocityCommand(Velocities *result);

/**
 * Obtains the user command (for tracking)
 * 
 * @param result A return parameter, which
 * will become the desired angle requested
 * by the user
 * 
 * @return 0 upon success, negative otherwise
 * @return An Angles structure, which reflects
 * the angle requested from the user
*/
int GetReferenceAngleCommand(Angles *result);

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
 * will become the position of the trolley
 * 
 * @return 0 upon success, other integers
 * if otherwise
 * @return A Positions structure, which
 * defines the Position of the Motor in
 * the lateral plane
*/
int GetTrolleyPosition(Positions *result);

/**
 * Obtains the Trolley Velocity
 * 
 * @param result A return parameter, which
 * will become the velocity of the trolley
 * 
 * @return 0 upon success, other integers
 * if otherwise
 * @return A Velocities structure, which
 * defines the velocity of the trolley in
 * the lateral plane
*/
int GetTrolleyVelocity(Velocities *result);

/**
 * Obtains the User Position
 * 
 * @param angle The rope angle
 * @param pos The trolley position
 * @param result A return parameter, which
 * will become the position of the user
 * 
 * @return 0 upon success, other integers
 * if otherwise
 * @return A Positions structure, which
 * defines the Position of the User in
 * the lateral plane
*/
int GetUserPosition(Angles *angle, Positions *pos, Positions *result);

/**
 * Obtains the User Velocity
 * 
 * @param angle The rope angle
 * @param vel The trolley velocity
 * @param result A return parameter, which
 * will become the velocity of the user
 * 
 * @return 0 upon success, other integers
 * if otherwise
 * @return A Velocities structure, which
 * defines the Velocity of the User in
 * the lateral plane
*/
int GetUserVelocity(Angles *angle, Velocities *vel, Velocities *result);


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