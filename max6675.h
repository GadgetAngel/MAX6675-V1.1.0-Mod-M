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

  MAX6675(uint32_t spi_cs, uint32_t spi_miso, uint32_t spi_sclk, uint8_t pin_mapping);
  MAX6675(uint32_t spi_cs, uint8_t pin_mapping);

  MAX6675(int8_t spi_cs, int8_t spi_miso, int8_t spi_sclk);
  MAX6675(int8_t spi_cs);

  void begin(void);
  float readCelsius(void);
  float readFahrenheit(void);
  uint16_t readRaw16(void);

  /*!    @brief  For compatibility with older versions
         @returns Temperature in F or NAN on failure! */
  float readFarenheit(void) { return readFahrenheit(); }

private:

  bool initialized;
  int8_t _sclk, _miso, _cs;
  uint32_t __sclk, __miso, __cs;
  uint8_t __pin_mapping = 0x00;
  bool first_reading = true;

  uint16_t spiread16(void);
};

#endif
