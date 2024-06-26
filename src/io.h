/**
 * @file io.h
 * @author Anti-Sway Team: Nguyen, Tri; Espinola, Malachi;
 * Tevy, Vattanary; Hokenstad, Ethan; Neff, Callen)
 * @brief Sensor/Actuator (Input/Output) Interfacing Library Header
 * @version 0.1
 * @date 2024-06-03
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef IO_H_
#define IO_H_

#include <stdbool.h>

#include "TimerIRQ.h"


/* Input/Output Data Types */

/// Alias for an Angle
typedef float Angle;
/// Alias for a Position
typedef float Position;
/// Alias for Velocity
typedef float Velocity;
/// Alias for Voltage
typedef float Voltage;


/**
 * @brief A 2D Angle
 * 
 * Defines the angle of the harness
 * along both directions, in radians
*/
typedef struct {
    ///! Angle Parallel to X Direction
    Angle x_angle;
    ///! Angle Parallel to Y Direction
    Angle y_angle;
} Angles;

/**
 * @brief A 2D Position
 * 
 * Defines the position of an object
 * in 2D space, in meters
*/
typedef struct {
    ///! X Position
    Position x_pos;
    ///! Y Position
    Position y_pos;
} Positions;

/**
 * @brief A 2D Velocity
 * 
 * Defines the velocity of an object
 * in 2D space, in meters/second
*/
typedef struct {
    ///! X Velocity
    Velocity x_vel;
    ///! Y Velocity
    Velocity y_vel;
} Velocities;

/* Sensor Variables */
/// The Timer
extern MyRio_IrqTimer timer;

/* Actuator Limits */
/// Motor Voltage High Limit (V)
#define MOTOR_V_LIM_H 10.000
/// Motor Voltage Low Limit (V)
#define MOTOR_V_LIM_L -10.000

/* Physical Parameters */
/// Pulley Radius (m)
#define R 0.0062
/// Current Constant (A/V)
#define K_a 0.41
/// Motor Constant (Nm/A)
#define K_m 0.11
/**
 * @brief Force to Voltage Conversion
 * 
 * @param force An int/float/double
 * expression, which represents the force
 * to transmit (through the motor)
 * 
 * @post Becomes the conversion between
 * force to the voltage to output
*/
#define FORCE_TO_VOLTAGE(force) \
    (force) * R / (K_a * K_m)
/**
 * @brief Voltage to Force Conversion
 * 
 * @param voltage An int/float/double
 * expression, which represents the voltage
 * to transmit (through the motor)
 * 
 * @post Converts voltage into a force
 */
#define VOLTAGE_TO_FORCE(voltage) \
	(voltage) * (K_a * K_m) / R

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


/* Reset Feature */


/**
 * Resets GetTrolleyPosition and
 * GetTrolleyVelocity by setting the
 * velocity to zero
 * 
 * @post The next time GetTrolleyVelocity
 * is called, both velocities are zero
*/
void Reset();


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
 * 
 * @pre This is called precisely once every BTI
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
 * 
 * @pre This is called precisely once every BTI
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


/* Keyboard Functions */

/**
 * Detects if the DEL key is pressed
 * on the keyboard
 * 
 * @return true iff DEL is pressed on
 * the keyboard
*/
bool PressedDelete();

/**
 * Enables Keyboard Control for Anti-Sway (concurrently)
 * 
 * @post If its already running, does nothing
 * 
 * @return 0 upon success, negative if error
*/
int KeyboardControlFork();

/**
 * Stops Keyboard Control for Anti-Sway (concurrent process)
 * 
 * @return 0 upon success, negative if error
*/
int KeyboardControlJoin();

#endif  /// IO_H_
