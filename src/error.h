/**
 * @file error.h
 * @author Anti-Sway Team: Nguyen, Tri; Espinola, Malachi;
 * Tevy, Vattanary; Hokenstad, Ethan; Neff, Callen)
 * @brief Universal Error Library
 * @version 0.1
 * @date 2024-06-03
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef ERROR_H_
#define ERROR_H_

/* Universal Error Codes */


/* Error Macro */
/// Ther universal error code
extern int u_error;


/* I/O Error Codes */

/// Unknown Exception
#define ENKWN -1
/// Out of Bounds Error
#define EOTBD -2
/// Velocity Exceeded Error
#define EVTYE -3
/// Angle Sensor Saturation Error
#define ESTRN -4
/// Encoder Error
#define EENCR -5


#endif  // ERROR_H_
