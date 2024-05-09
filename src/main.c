// Copyright 2024 Anti-Sway Team (Nguyen, Tri; Espinola, Malachi;
// Tevy, Vattanary; Hokenstad, Ethan; Neff, Callen)

#include <stdlib.h>


#include "thread-lib.h"
#include "system.h"

int main(int argc, char **argv) {
	int status;
	VERIFY(status, SystemExec());
	return EXIT_SUCCESS;
}
