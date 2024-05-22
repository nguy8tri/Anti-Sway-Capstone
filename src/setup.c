// Copyright 2024 Anti-Sway Team (Nguyen, Tri; Espinola, Malachi;
// Tevy, Vattanary; Hokenstad, Ethan; Neff, Callen)

#include <stdlib.h>

#include "MyRio.h"
#include "T1.h"

#include "record.h"
#include "io.h"
#include "error.h"

#include "setup.h"


// Local error flag
static int error;


/* Declaration of universal error code */


// Universal error code (extern)
int u_error;

int Setup() {
    u_error = 0;
    if (MyRio_IsNotSuccess(MyRio_Open())) return EXIT_FAILURE;
    VERIFY(error, IOSetup());
    // VERIFY(error, EncoderFork());
    return EXIT_SUCCESS;
}

int Shutdown() {
    VERIFY(error, IOShutdown());
    VERIFY(error, SaveDataFiles());
    // VERIFY(error, EncoderJoin());
    return MyRio_Close();
}
