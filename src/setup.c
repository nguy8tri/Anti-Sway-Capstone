// Copyright 2024 Anti-Sway Team (Nguyen, Tri; Espinola, Malachi;
// Tevy, Vattanary; Hokenstad, Ethan; Neff, Callen)

#include <stdlib.h>

#include "MyRio.h"
#include "T1.h"

#include "record.h"
#include "io.h"

#include "setup.h"

static int error;

int Setup() {
    if (MyRio_IsNotSuccess(MyRio_Open())) return EXIT_FAILURE;
    VERIFY(error, IOSetup());
    return EXIT_SUCCESS;
}

int Shutdown() {
    VERIFY(error, IOShutdown());
    VERIFY(error, SaveDataFiles());
    return MyRio_Close();
}
