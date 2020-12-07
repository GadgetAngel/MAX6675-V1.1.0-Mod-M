// this library is public domain. enjoy!
// https://learn.adafruit.com/thermocouple/

#ifndef ADAFRUIT_MAX6675_H
#define ADAFRUIT_MAX6675_H

#if (ARDUINO >= 100 || ARDUINOLPC)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/**************************************************************************/
/*!
    @brief  Class for communicating with thermocouple sensor
*/
/**************************************************************************/
class MAX6675 {
public:
  double Last_read_temp=0;
  MAX6675(int8_t SCLK, int8_t CS, int8_t MISO);

  float readCelsius(void);
  float readFahrenheit(void);
  uint16_t readRaw16(void);

  /*!    @brief  For compatibility with older versions
         @returns Temperature in F or NAN on failure! */
  float readFarenheit(void) { return readFahrenheit(); }

private:
  unsigned long Last_read_time=0;
  int8_t sclk, miso, cs;
  uint8_t spiread(void);
};

#endif
