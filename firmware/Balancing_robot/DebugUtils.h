
/*
DebugUtils.h - Simple debugging utilities.

Ideas taken from:
http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1271517197

*/

#ifndef DEBUGUTILS_H
#define DEBUGUTILS_H

#ifdef DEBUG

#include <Arduino.h>

#define DEBUG_PRINT(str)    \
   Serial.print(millis());     \
   Serial.print(": ");    \
   Serial.print(__PRETTY_FUNCTION__); \
   Serial.print(' ');      \
   Serial.print(__FILE__);     \
   Serial.print(':');      \
   Serial.print(__LINE__);     \
   Serial.print(' ');      \
   Serial.println(str);
#else
#define DEBUG_PRINT(str)
#endif

#endif

