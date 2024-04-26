// Copyright 2024 Tri Nguyen

#include "MyRio.h"
#include "T1.h"

int main(int argc, char **argv) {
    NiFpga_Status status;

    status = MyRio_Open();  // open FPGA session
    if (MyRio_IsNotSuccess(status)) return status;

    if (PIDControllerExec()) return EXIT_FAILURE;

    status = MyRio_Close();  // close FPGA session

    return status;
}
