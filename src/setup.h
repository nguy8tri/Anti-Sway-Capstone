// Copyright 2024 Anti-Sway Team (Nguyen, Tri; Espinola, Malachi;
// Tevy, Vattanary; Hokenstad, Ethan; Neff, Callen)

#ifndef SETUP_H_
#define SETUP_H_


/* Error Macro (for setup/shutdown) */


#define VERIFY(code, statement) \
    if (code = statement) return code

/* Global Setup/Shutdown Functions */


/**
 * Sets up the entire System
 * 
 * @return 0 upon success, negative
 * otherwise
*/
int Setup();


/**
 * Shuts the entire System down
 * 
 * @return 0 upon success, negative
 * otherwise
*/
int Shutdown();

#endif  // SETUP_H_
