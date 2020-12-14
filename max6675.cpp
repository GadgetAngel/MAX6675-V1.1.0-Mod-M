// this library is public domain. enjoy!
// https://learn.adafruit.com/thermocouple/

#include "max6675.h"

#ifndef __AVR
  #include "../../../../Marlin/src/HAL/shared/HAL_SPI.h"
#endif

#include "../../../../Marlin/src/HAL/shared/Delay.h"

#ifdef __AVR
  #include <avr/pgmspace.h>
#elif defined(ESP8266)
  #include <pgmspace.h>
#endif

#include <stdlib.h>
#include <SPI.h>

#ifdef __AVR
  static SPISettings max6675_spisettings = 
      SPISettings(4000000, MSBFIRST, SPI_MODE0);    
#else
  static SPISettings max6675_spisettings =
      SPISettings(SPI_QUARTER_SPEED, MSBFIRST, SPI_MODE0);
#endif

/**************************************************************************/
/*!
    @brief Create the interface object using software (bitbang) SPI
    @param _cs the SPI CS pin to use 
    @param _miso the SPI MISO pin to use
    @param _sclk the SPI clock pin to use   
*/
/**************************************************************************/
MAX6675::MAX6675(int8_t _cs, int8_t _miso, int8_t _sclk) {
  cs = _cs;
  miso = _miso;
  sclk = _sclk;
  
  initialized = false;
}

/**************************************************************************/
/*!
    @brief Create the interface object using hardware SPI
    @param _cs the SPI CS pin to use
*/
/**************************************************************************/
MAX6675::MAX6675(int8_t _cs) {
  cs = _cs;
  sclk = miso = -1;

  initialized = false;
}

/**************************************************************************/
/*!
    @brief Initialize the SPI interface 
    @return True
*/
/**************************************************************************/
void MAX6675::begin(void) {

  //define pin modes
  pinMode(cs, OUTPUT);
  digitalWrite(cs, HIGH);

  if (sclk == -1) {
    // hardware SPI

      SPI.begin();
  } 
  else {
    pinMode(sclk, OUTPUT); 
    pinMode(miso, INPUT);
  }
  initialized = true;
}

/**************************************************************************/
/*!
    @brief  Read the Celsius temperature (supports negative tempatures)
    @returns Temperature in C or NAN on failure!
*/
/**************************************************************************/
float MAX6675::readCelsius(void) {

  uint16_t v;

  v = readRaw16();

  if (v & 0x4) {
    // uh oh, no thermocouple attached!
    return NAN;
  }

  if (v & 0x8000) {
    // Negative value, drop the lower 3 bits and explicitly extend sign bits.
    v = 0xE000 | ((v >> 3) & 0x1FFF);
  }
  else {
    // Positive value, just drop the lower 3 bits.
    v >>= 3;
  }

  float centigrade = v;

  centigrade *= 0.25;

  return centigrade;
}

/**************************************************************************/
/*!
    @brief  Read the Fahenheit temperature
    @returns Temperature in F or NAN on failure!
*/
/**************************************************************************/
float MAX6675::readFahrenheit(void) { return readCelsius() * 9.0 / 5.0 + 32; }

/**************************************************************************/
/*!
    @brief  Read the raw data packet for the unsigned 16 bits 
    that represents the temperature
    @returns Raw value read in 16 bits!
*/
/**************************************************************************/
uint16_t MAX6675::readRaw16(void) {
  int i;
  uint16_t v = 0;

 // backcompatibility!
  if (! initialized) {
    begin();
  }

  //enable the SPI communication
  digitalWrite(cs, LOW);
  DELAY_US(1000); 

  if (sclk == -1) {
    // hardware SPI

    SPI.beginTransaction(max6675_spisettings);

    v = SPI.transfer(0);
    v <<= 8;
    v |= SPI.transfer(0);
   
    SPI.endTransaction();
  } 
  else {

    digitalWrite(sclk, LOW);
    DELAY_US(1000);    

    for (i = 15; i >= 0; i--) {
      digitalWrite(sclk, LOW);
      DELAY_US(1000);
      v <<= 1;
      if (digitalRead(miso)) {
	      v |= 1;
      }
      
      digitalWrite(sclk, HIGH);
      DELAY_US(1000);
    }
  }
  //disable SPI communication
  digitalWrite(cs, HIGH);

  return v; 
 }