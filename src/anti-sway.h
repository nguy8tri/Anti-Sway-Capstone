// Copyright 2024 Anti-Sway Team (Nguyen, Tri; Espinola, Malachi;
// Tevy, Vattanary; Hokenstad, Ethan; Neff, Callen)

#ifndef ANTI_SWAY_H_
#define ANTI_SWAY_H_

/* Execution-Dispatch Function */


/**
 * Executes Anti-Sway Mode (concurrently)
 * 
 * @post If its already running, does nothing
*/
int AntiSwayFork();

/**
 * Stops Anti-Sway Mode (concurrent process)
*/
int AntiSwayJoin();

#endif  // ANTI_SWAY_H_
