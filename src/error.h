// Copyright 2024 Anti-Sway Team (Nguyen, Tri; Espinola, Malachi;
// Tevy, Vattanary; Hokenstad, Ethan; Neff, Callen)

#ifndef ERROR_H_
#define ERROR_H_

/* Universal Error Codes */


/* Error Macro */
// Ther universal error code
extern int u_error;


/* I/O Error Codes */

// Unknown Exception
#define ENKWN -1
// Out of Bounds Error
#define EOTBD -2
// Velocity Exceeded Error
#define EVTYE -3
// Unexpected Saturation Error
#define ESTRN -4


#endif  // ERROR_H_
