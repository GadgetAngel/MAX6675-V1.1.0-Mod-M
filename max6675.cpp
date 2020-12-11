// this library is public domain. enjoy!
// https://learn.adafruit.com/thermocouple/

#include "max6675.h"
#include <SPI.h>
#include <stdlib.h>
#ifdef __AVR
  #include <avr/pgmspace.h>
#elif defined(ESP8266)
  #include <pgmspace.h>
#endif

/**************************************************************************/
/*!
    @brief Create the interface object using software (bitbang) SPI
    @param _cs the SPI CS pin to use
    @param _sclk the SPI clock pin to use    
    @param _miso the SPI MISO pin to use
*/
/**************************************************************************/
MAX6675::MAX6675(int8_t _cs, int8_t _sclk, int8_t _miso) {
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
    //start and configure hardware SPI
    SPI.begin();
  } else {
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

  v = spiread();
  v <<= 8;
  v |= spiread();

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
    @brief  Read the Raw value of unsigned 16 bits for the temperature
    @returns Raw value read in 16 bits!
*/
/**************************************************************************/
uint16_t MAX6675::readRaw16(void) {

  uint16_t v;

  // try sending back same temperature if trying
  // to read faster than MAX6675 likes
  // see if this avoids 0 being sent back
  //if(Last_read_time + 500UL > millis())
    //return Last_read_temp;

  //Last_read_time = millis();

 // backcompatibility!
  if (! initialized) {
    begin();
  }

  if (sclk == -1) {
    // hardware SPI

    SPI.beginTransaction(SPISettings(SPI_QUARTER_SPEED, MSBFIRST, SPI_MODE0));
    //SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));

    v = spiread();
    v <<= 8;
    v |= spiread();

    SPI.endTransaction();
  } 
  else {

    // software SPI
    digitalWrite(cs, LOW);
    //delay(1);
    delayMicroseconds(10);   

    v = spiread();
    v <<= 8;
    v |= spiread();

    digitalWrite(cs, HIGH);
    //Serial.println(d, HEX);

  }

  //Last_read_temp = v;

  return v; 

 }


/**********************************************/

uint8_t MAX6675::spiread(void) {
  int i;
  uint8_t d = 0;
  
  if(sclk == -1) {
    // hardware SPI

    d = SPI.transfer(0);

  } 
  else {
    // software SPI

    for (i = 7; i >= 0; i--) {
    digitalWrite(sclk, LOW);
    delayMicroseconds(10);
    if (digitalRead(miso)) {
      // set the bit to 0 no matter what
      d |= (1 << i);
    }

      digitalWrite(sclk, HIGH);
      //delay(1);
      delayMicroseconds(10);      
    }
  }

  return d;
}
