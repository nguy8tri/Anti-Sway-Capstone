// Copyright 2024 Anti-Sway Team (Nguyen, Tri; Espinola, Malachi;
// Tevy, Vattanary; Hokenstad, Ethan; Neff, Callen)

#include "MyRio.h"
#include "T1.h"

#include "io.h"

#include "setup.h"

static int error;

int Setup() {
    if (MyRio_IsNotSuccess(MyRio_Open())) return status;
    VERIFY(error, IOSetup());
}

int Shutdown() {
    VERIFY(error, IOShutdown());
    return MyRio_Close();
}
