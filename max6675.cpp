// this library is public domain. enjoy!
// https://learn.adafruit.com/thermocouple/

//#define DEBUG_STM32
//#define DEBUG_STM32_SPI
//#define DEBUG_LPC_SPI
//#define DEBUG_LPC

#if defined(ARDUINO_ARCH_STM32) && defined(DEBUG_STM32)
  #define HAS_STM32_DEBUG 1
#endif

#if defined(ARDUINO_ARCH_STM32) && defined(DEBUG_STM32_SPI)
  #define HAS_STM32_DEBUG_SPI 1
#endif

#if defined(ARDUINO_ARCH_LPC176X) && defined(DEBUG_LPC)
  #define HAS_LPC1768_DEBUG 1
#endif

#if defined(ARDUINO_ARCH_LPC176X) && defined(DEBUG_LPC_SPI)
  #define HAS_LPC1768_DEBUG_SPI 1
#endif

#if HAS_LPC1768_DEBUG || HAS_LPC1768_DEBUG_SPI
  #include "../../../../Marlin/src/inc/MarlinConfig.h"
#endif

#include "max6675.h"

#include "../../../../Marlin/src/HAL/shared/Delay.h"

#ifdef __AVR__
  #include <avr/pgmspace.h>
#elif defined(ESP8266)
  #include <pgmspace.h>
#endif

#include <stdlib.h>
#include <SPI.h>

/**
 * SPI speed where 0 <= index <= 6
 *
 * Approximate rates :
 *
 *  0 :  8 - 10 MHz
 *  1 :  4 - 5 MHz
 *  2 :  2 - 2.5 MHz
 *  3 :  1 - 1.25 MHz
 *  4 :  500 - 625 kHz
 *  5 :  250 - 312 kHz
 *  6 :  125 - 156 kHz
 *
 *  On AVR, actual speed is F_CPU/2^(1 + index).
 *  On other platforms, speed should be in range given above where possible.
 */
#if !defined(__AVR__)
  #define SPI_FULL_SPEED      0   // Set SCK to max rate
  #define SPI_HALF_SPEED      1   // Set SCK rate to half of max rate
  #define SPI_QUARTER_SPEED   2   // Set SCK rate to quarter of max rate
  #define SPI_EIGHTH_SPEED    3   // Set SCK rate to 1/8 of max rate
  #define SPI_SIXTEENTH_SPEED 4   // Set SCK rate to 1/16 of max rate
  #define SPI_SPEED_5         5   // Set SCK rate to 1/32 of max rate
  #define SPI_SPEED_6         6   // Set SCK rate to 1/64 of max rate
#endif

#ifdef __AVR__
  static SPISettings max6675_spisettings =
      SPISettings(4000000, MSBFIRST, SPI_MODE0);
#else
  static SPISettings max6675_spisettings =
      SPISettings(SPI_QUARTER_SPEED, MSBFIRST, SPI_MODE0);
#endif

/**************************************************************************/
/*!
    @brief Create the interface object using software (bitbang) SPI for
    PIN values which are larger than 127. If you have PIN values less than
    or equal to 127 use the other call for SW SPI.
    @param spi_cs the SPI CS pin to use
    @param spi_miso the SPI MISO pin to use
    @param spi_sclk the SPI clock pin to use
    @param pin_mapping set to 1 for positive pin values
*/
/**************************************************************************/
MAX6675::MAX6675(uint32_t spi_cs, uint32_t spi_miso, uint32_t spi_sclk,
                 uint8_t pin_mapping) {
  __cs = spi_cs;
  __miso = spi_miso;
  __sclk = spi_sclk;
  __pin_mapping = pin_mapping;

  if (__pin_mapping == 0) {
    _cs = __cs;
    _miso = __miso;
    _sclk = __sclk;
  }

  initialized = false;
}

/**************************************************************************/
/*!
    @brief Create the interface object using hardware SPI for PIN values
    which are larger than 127. If you have PIN values less than
    or equal to 127 use the other call for HW SPI
    @param spi_cs the SPI CS pin to use
    @param pin_mapping set to 1 for positive pin values
*/
/**************************************************************************/
MAX6675::MAX6675(uint32_t spi_cs, uint8_t pin_mapping) {
  __cs = spi_cs;
  __sclk = __miso = -1UL;  //-1UL or 0xFFFFFFFF
  __pin_mapping = pin_mapping;

  if (__pin_mapping == 0) {
    _cs = __cs;
    _miso = -1;
    _sclk = -1;
  }

  initialized = false;
}

/**************************************************************************/
/*!
    @brief Create the interface object using software (bitbang) SPI for PIN
    values less than or equal to 127.
    @param spi_cs the SPI CS pin to use
    @param spi_miso the SPI MISO pin to use
    @param spi_sclk the SPI clock pin to use
*/
/**************************************************************************/
MAX6675::MAX6675(int8_t spi_cs, int8_t spi_miso, int8_t spi_sclk) {
  _cs = spi_cs;
  _miso = spi_miso;
  _sclk = spi_sclk;

  initialized = false;
}

/**************************************************************************/
/*!
    @brief Create the interface object using hardware SPI for PIN
    values less than or equal to 127.
    @param spi_cs the SPI CS pin to use
*/
/**************************************************************************/
MAX6675::MAX6675(int8_t spi_cs) {
  _cs = spi_cs;
  _sclk = _miso = -1;

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
  if (!__pin_mapping) {
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);
  }
  else {
    pinMode(__cs, OUTPUT);
    digitalWrite(__cs, HIGH);
  }

  if (_sclk == -1 || __sclk == -1UL) {
    // hardware SPI

      SPI.begin();
  }
  else {
   if (!__pin_mapping) {
      pinMode(_sclk, OUTPUT);
      pinMode(_miso, INPUT);
   }
   else {
      pinMode(__sclk, OUTPUT);
      pinMode(__miso, INPUT);
   }
  }

  #if HAS_STM32_DEBUG_SPI
    if (!__pin_mapping) {
      Serial.print("\n\n_cs: ");
      Serial.print(_cs);
      Serial.print(" _cs: 0x");
      Serial.print(_cs, HEX);
      uint32_t cs2 = 0;
      cs2 = _cs;
      Serial.print("\n uint32_t cs2: ");
      Serial.print(cs2);
      Serial.print("  cs2: 0x");
      Serial.print(cs2, HEX);
      Serial.print("\n _miso: ");
      Serial.print(_miso);
      Serial.print(" _miso: 0x");
      Serial.print(_miso, HEX);
      Serial.print("\n _sclk: ");
      Serial.print(_sclk);
      Serial.print(" _sclk: 0x");
      Serial.print(_sclk, HEX);
      Serial.print("\n\n");
    }
    else {
      Serial.print("\n\n__cs: ");
      Serial.print(__cs);
      Serial.print(" __cs: 0x");
      Serial.print(__cs, HEX);
      Serial.print("\n __miso: ");
      Serial.print(__miso);
      Serial.print(" __miso: 0x");
      Serial.print(__miso, HEX);
      Serial.print("\n __sclk: ");
      Serial.print(__sclk);
      Serial.print(" __sclk: 0x");
      Serial.print(__sclk, HEX);
      Serial.print("\n __pin_mapping: ");
      Serial.print(__pin_mapping);
      Serial.print("\n\n");
    }
  #endif

  #if HAS_LPC1768_DEBUG_SPI
    // for testing
    if (!__pin_mapping) {
      SERIAL_ECHOLN();
      SERIAL_ECHOLNPAIR("Regular call for _cs: ", _cs ," _miso: ", _miso ," _sclk: ", _sclk);
      SERIAL_PRINTF("Regular call for _cs: 0x%X  _miso: 0x%X  _sclk: 0x%X  ", _cs, _miso, _sclk);
      SERIAL_ECHOLN();
      SERIAL_ECHOLN();
    }
    else {
      SERIAL_ECHOLN();
      SERIAL_ECHOLNPAIR("PIN_MAPPING call for __cs: ", __cs ," __miso: ", __miso ," __sclk: ", __sclk);
      SERIAL_PRINTF("PIN_MAPPING call for __cs: 0x%X  __miso: 0x%X  __sclk: 0x%X  ", __cs, __miso, __sclk);
      SERIAL_ECHOLN();
      SERIAL_ECHOLN();
    }
  #endif

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

  if (!first_reading)
    v = spiread16();
  else {
    first_reading = false;
    v = spiread16();
    #if HAS_STM32_DEBUG
      Serial.print("\n\n1st Reading: 0x");
      Serial.println(v, HEX);
    #endif
    #if HAS_LPC1768_DEBUG
      SERIAL_ECHOLN();
      SERIAL_ECHOLN();
      SERIAL_ECHO("1st Reading:");
      SERIAL_PRINTF("   0x%X  ", v);
      SERIAL_ECHOLN();
    #endif
    v = spiread16();
    #if HAS_STM32_DEBUG
      Serial.print("\n\n2nd Reading: 0x");
      Serial.println(v, HEX);
    #endif
    #if HAS_LPC1768_DEBUG
      SERIAL_ECHOLN();
      SERIAL_ECHOLN();
      SERIAL_ECHO("2nd Reading:");
      SERIAL_PRINTF("   0x%X  ", v);
      SERIAL_ECHOLN();
    #endif
  }

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
  uint16_t d = 0;

  if (!first_reading)
    return spiread16();
  else {
    first_reading = false;
    d = spiread16();
    #if HAS_STM32_DEBUG
      Serial.print("\n\n1st Reading: 0x");
      Serial.println(d, HEX);
    #endif
    #if HAS_LPC1768_DEBUG
      SERIAL_ECHOLN();
      SERIAL_ECHOLN();
      SERIAL_ECHO("1st Reading:");
      SERIAL_PRINTF("   0x%X  ", d);
      SERIAL_ECHOLN();
    #endif
    d = spiread16();
    #if HAS_STM32_DEBUG
      Serial.print("\n\n2nd Reading: 0x");
      Serial.println(d, HEX);
    #endif
    #if HAS_LPC1768_DEBUG
      SERIAL_ECHOLN();
      SERIAL_ECHOLN();
      SERIAL_ECHO("2nd Reading:");
      SERIAL_PRINTF("   0x%X  ", d);
      SERIAL_ECHOLN();
    #endif
    return d;
  }
}

/**************************************************************************/

uint16_t MAX6675::spiread16(void) {
  int i;
  #if HAS_STM32_DEBUG
    int read_v = 0;
  #endif
  uint16_t v = 0;

 // backcompatibility!
  if (! initialized) {
    begin();
  }

  //enable the SPI communication
  if (!__pin_mapping)
    digitalWrite(_cs, LOW);
  else
    digitalWrite(__cs, LOW);
  DELAY_US(1000);

  if (_sclk == -1 || __sclk == -1UL) {
    // hardware SPI

    SPI.beginTransaction(max6675_spisettings);

    v = SPI.transfer(0);
    v <<= 8;
    v |= SPI.transfer(0);

    SPI.endTransaction();
  }
  else {

    if (!__pin_mapping)
      digitalWrite(_sclk, LOW);
    else
      digitalWrite(__sclk, LOW);
    DELAY_US(1000);

    #if HAS_STM32_DEBUG
      Serial.print("\n\nBEGINING of NEW 16-bit number: 0b");
    #endif

    for (i = 15; i >= 0; i--) {
      if (!__pin_mapping)
        digitalWrite(_sclk, LOW);
      else
        digitalWrite(__sclk, LOW);
      DELAY_US(1000);

      v <<= 1;

      if (!__pin_mapping) {
        #if HAS_STM32_DEBUG
          read_v = digitalRead(_miso);
          Serial.print(read_v, HEX);
          if (read_v) {
        #else
          if (digitalRead(_miso)) {
        #endif
	        v |= 1;
        }
      }
      else {
        #if HAS_STM32_DEBUG
          read_v = digitalRead(__miso);
          Serial.print(read_v, HEX);
          if (read_v) {
        #else
          if (digitalRead(__miso)) {
        #endif
	        v |= 1;
        }
      }


      if (!__pin_mapping)
        digitalWrite(_sclk, HIGH);
      else
        digitalWrite(__sclk, HIGH);
      DELAY_US(1000);
    }
  }

  //disable SPI communication
  if (!__pin_mapping)
    digitalWrite(_cs, HIGH);
  else
    digitalWrite(__cs, HIGH);

  #if HAS_STM32_DEBUG || HAS_LPC1768_DEBUG
    uint16_t v2 = v >> 3;
  #endif
  #if HAS_STM32_DEBUG
    Serial.print("\nv >> 3 : 0b");
    Serial.print(v2, BIN);
    Serial.print("  : 0x");
    Serial.print(v2, HEX);
    Serial.print("  : ");
    Serial.println(v2);
    Serial.println(" ");
  #endif

  #if HAS_LPC1768_DEBUG
    SERIAL_ECHOLN();
    SERIAL_ECHOLN("v >> 3: 0b");
    print_bin(v2);
    SERIAL_PRINTF("  : 0x%X  ", v2);
    SERIAL_ECHOLNPAIR(" : ", v2);
    SERIAL_ECHOLN();
  #endif

  return v;
 }