#ifndef _IO_H_
#define _IO_H_


/* Input/Output Data Types */


/**
 * Defines the angle of the harness
 * along both directions, in radians
*/
typedef struct {
    float x_angle;
    float y_angle;
} Angle;

/**
 * Defines the position of an object
 * in 2D space, in meters
*/
typedef struct {
    float x_pos;
    float y_pos;
} Position;

typedef float Voltage;


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
 * Obtains the angle of the harness
 * 
 * @return An Angle structure, which
 * defines the angle of the harness along
 * both lateral directions
*/
Angle GetAngle();

/**
 * Obtains the Motor Position
 * 
 * @return A Position structure, which
 * defines the Position of the Motor in
 * the lateral plane
*/
Position GetMotorPosition();

/**
 * Obtains the User Position
 * 
 * @return A Position structure, which
 * defines the Position of the User in
 * the lateral plane
*/
Position GetUserPosition();


/* Actuator Functions */


/**
 * Sets the voltage of the motor
 * 
 * @return 0 upon success, other integers
 * if otherwise
*/
int SetVoltage(Voltage voltage);

#endif