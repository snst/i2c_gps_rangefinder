// Copyright 2018 Stefan Schmidt

#include "config.h"
#include "ndebug.h"
#include <arduino.h>
#include <SoftwareSerial.h>

#ifdef DEBUG_SERIAL
# ifdef GPS
  SoftwareSerial debugSerial(2, 3); // RX, TX 
# endif
#endif
