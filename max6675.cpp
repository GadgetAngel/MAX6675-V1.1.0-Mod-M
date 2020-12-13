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

#include <SPI.h>
#include <stdlib.h>

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
  sclk = _sclk;
  cs = _cs;
  miso = _miso;

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

    #if TARGET_LPC1768
    //The SPI interface for LPC176x is incomplete, therefore
    // Use Marlin's calls for hardware SPI 
      spiBegin();
      spiInit(SPI_QUARTER_SPEED);
    #else
      SPI.begin();
    #endif
  } 
  else {
    pinMode(sclk, OUTPUT); 
    pinMode(miso, INPUT);
  }
  initialized = true;
}

/**************************************************************************/
/*!
    @brief  Read the Celsius temperature
    @returns Temperature in C or NAN on failure!
*/
/**************************************************************************/
float MAX6675::readCelsius(void) {

  uint16_t v;

  v = readRaw16();

  if (v & 0x4) {
    // uh oh, no thermocouple attached!
    return NAN;
    // return -100;
  }

  v >>= 3;

  return v * 0.25;
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

  uint16_t v;

 // backcompatibility!
  if (! initialized) {
    begin();
  }

  //enable the SPI communication
  digitalWrite(cs, LOW);
  DELAY_NS(100);  

  if (sclk == -1) {
    // hardware SPI
    #if defined(__AVR) || !TARGET_LPC1768
      SPI.beginTransaction(max6675_spisettings);
    #endif  

    v = spiread();
    v <<= 8;
    v |= spiread();

    #if defined(__AVR) || !TARGET_LPC1768   
      SPI.endTransaction();
    #endif
  } 
  else {
    //Software SPI
    v = spiread();
    v <<= 8;
    v |= spiread();
  }

  //disable SPI communication
  digitalWrite(cs, HIGH);

  return v; 

 }


/**********************************************/

uint8_t MAX6675::spiread(void) {
  int i;
  uint8_t d = 0;
  
  if(sclk == -1) {
    // hardware SPI
    #if TARGET_LPC1768
      d = spiRec();      
    #else
      d = SPI.transfer(0);
    #endif   
  } 
  else {
    // software SPI
    for (i = 7; i >= 0; i--) {
    digitalWrite(sclk, LOW);
    DELAY_US(10);
    if (digitalRead(miso)) {
      // set the bit to 0 no matter what
      d |= (1 << i);
    }
      digitalWrite(sclk, HIGH);
      DELAY_US(10);     
    }
  }

  return d;
}
