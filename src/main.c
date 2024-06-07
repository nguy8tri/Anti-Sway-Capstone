/**
 * @file main.c
 * @author Anti-Sway Team: Nguyen, Tri; Espinola, Malachi;
 * Tevy, Vattanary; Hokenstad, Ethan; Neff, Callen)
 * @brief Main File
 * @version 0.1
 * @date 2024-06-03
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <stdlib.h>


#include "thread-lib.h"
#include "system.h"

/**
 * Runs the Anti-Sway Capstone Project
 * 
 * @param argc Command Line Arguments (Quantity)
 * @param argv Command Line Arguments (Contents)
 * 
 * @return 0 iff success, negative otherwise
 */
int main(int argc, char **argv) {
	int status;
	VERIFY(status, SystemExec());
	return EXIT_SUCCESS;
}
