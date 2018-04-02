#ifndef NDEBUG_H_
#define NDEBUG_H_
#include "config.h"
#include <SoftwareSerial.h>

//#define DEBUG_SERIAL

#ifdef DEBUG_SERIAL

# ifdef GPS
  extern SoftwareSerial debugSerial;
# endif
# ifdef SONAR
# define debugSerial Serial
# endif

# define _D(x) debugSerial.println(x)
# define _d(x) debugSerial.print(x)
# define _debugInit() {debugSerial.begin(9600);}
#else
# define _D(x)
# define _d(x)
# define _debugInit()
#endif


#endif

