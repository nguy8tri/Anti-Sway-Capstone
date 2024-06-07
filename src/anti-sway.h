/**
 * @file anti-sway.h
 * @author Anti-Sway Team: Nguyen, Tri; Espinola, Malachi;
 * Tevy, Vattanary; Hokenstad, Ethan; Neff, Callen)
 * @brief Anti-Sway Control Law Header
 * @version 0.1
 * @date 2024-06-03
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef ANTI_SWAY_H_
#define ANTI_SWAY_H_

/* Execution-Dispatch Function */


/**
 * @brief Executes Anti-Sway Mode
 * 
 * Executes Anti-Sway Mode (concurrently)
 * 
 * @pre Anti-Sway Mode is not already running
*/
int AntiSwayFork();

/**
 * @brief Stops Anti-Sway Mode
 * 
 * Stops Anti-Sway Mode (concurrent process)
*/
int AntiSwayJoin();

#endif  // ANTI_SWAY_H_
