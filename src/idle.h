// Copyright 2024 Anti-Sway Team (Nguyen, Tri; Espinola, Malachi;
// Tevy, Vattanary; Hokenstad, Ethan; Neff, Callen)

#ifndef IDLE_H_
#define IDLE_H_

/**
 * Executes Idle Mode (Concurrently)
 *
 * @post If its already running, does nothing
 * 
 * @return 0 upon success, negative if error
*/
int IdleFork();

/**
 * Stops Idle Mode (concurrent process)
 * 
 * @return 0 upon success, negative if error
*/
int IdleJoin();

#endif  // IDLE_H_