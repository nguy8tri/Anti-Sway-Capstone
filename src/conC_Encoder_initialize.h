//
//  conC_Encoder_initialize.h
//  
//
//  Created by JOSEPH L GARBINI on 12/28/17.
//

#ifndef conC_Encoder_initialize_h
#define conC_Encoder_initialize_h

#include <stdio.h>
#include "MyRio.h"
#include "Encoder.h"

NiFpga_Status    conC_Encoder_initialize(NiFpga_Session myrio_session, MyRio_Encoder *encCp, int iE);

#endif /* conC_Encoder_initialize_h */
