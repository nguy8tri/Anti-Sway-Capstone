// Copyright 2024 Anti-Sway Team (Nguyen, Tri; Espinola, Malachi;
// Tevy, Vattanary; Hokenstad, Ethan; Neff, Callen)

#ifndef IDLE_H_
#define IDLE_H_

/**
 * Executes Idle Mode (concurrently), so we see
 * how badly we messed up our code/sensors
 *
 * @post If its already running, does nothing
 * 
 * @return 0 upon success, negative if error
*/
int IdleFork();

/**
 * Stops Idle Mode (concurrent process) and our
 * pain
 * 
 * @return 0 upon success, negative if error
*/
int IdleJoin();

#endif  // IDLE_H_