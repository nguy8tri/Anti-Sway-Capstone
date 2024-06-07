/**
 * @file tracking.h
 * @author Anti-Sway Team: Nguyen, Tri; Espinola, Malachi;
 * Tevy, Vattanary; Hokenstad, Ethan; Neff, Callen)
 * @brief Tracking Mode Control Law Header
 * @version 0.1
 * @date 2024-06-03
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef TRACKING_H_
#define TRACKING_H_


/* Execution-Dispatch Function */


/**
 * Executes Tracking Mode (concurrently)
 * 
 * @pre Tracking Mode is not already running
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
