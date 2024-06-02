// Copyright 2024 Anti-Sway Team (Nguyen, Tri; Espinola, Malachi;
// Tevy, Vattanary; Hokenstad, Ethan; Neff, Callen)

#ifndef TRACKING_H_
#define TRACKING_H_


/* Execution-Dispatch Function */


/**
 * Executes Tracking Mode (concurrently)
 * 
 * @post If its already running, does nothing
 * 
 * @return 0 upon success, negative if error
*/
int TrackingFork();

/**
 * Stops Tracking Mode (concurrent process)
 * 
 * @return 0 upon success, negative if error
*/
int TrackingJoin();

#endif  // TRACKING_H_
