/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
#include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#include "gbg.h"

/*=========================================================================
  APPLICATION SETTINGS

  FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
   Enabling this will put your Bluefruit LE module
   in a 'known good' state and clear any config
   data set in previous sketches or projects, so
   running this at least once is a good idea.
   
   When deploying your project, however, you will
   want to disable factory reset by setting this
   value to 0.  If you are making changes to your
   Bluefruit LE device via AT commands, and those
   changes aren't persisting across resets, this
   is the reason why.  Factory reset will erase
   the non-volatile memory where config data is
   stored, setting it back to factory default
   values.
       
       Some sketches that require you to bond to a
       central device (HID mouse, keyboard, etc.)
       won't work at all with this feature enabled
       since the factory reset will clear all of the
       bonding data stored on the chip, meaning the
       central device won't be able to reconnect.
       MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
       MODE_LED_BEHAVIOUR        LED activity, valid options are
       "DISABLE" or "MODE" or "BLEUART" or
       "HWUART"  or "SPI"  or "MANUAL"
       -----------------------------------------------------------------------*/
#define FACTORYRESET_ENABLE         0 /* tbd temp to test bd without FR */
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

// Create the bluefruit object, either software serial...uncomment these lines
/*
  SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

  Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
  BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);
/**************************************************************************
 **************************************************************************/

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */

//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
extern Adafruit_BluefruitLE_SPI ble;


/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
// tbd mje changed code to return after error instead of hang
void error(const __FlashStringHelper*err) {
    Serial.println(err);
    //return(2);
    while (1);                  /* dead code */
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];

/**************************************************************************/
/*!
  @brief  Sets up the HW an the BLE module (this function is called
  automatically on startup)
*/
/**************************************************************************/
void setupBT(void)
{
    int countdown = 0;

       /* Initialise the module */
    Serial.print(F("Initialising the Bluefruit LE module: "));

    if ( !ble.begin(VERBOSE_MODE) )
    {
        error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
    }
    Serial.println( F("OK!") );

    if ( FACTORYRESET_ENABLE )
    {
           /* Perform a factory reset to make sure everything is in a known state */
        Serial.println(F("Performing a factory reset: "));
        if ( ! ble.factoryReset() ){
            
            error(F("Couldn't factory reset"));
        }
    }
       /* Disable command echo from Bluefruit */
    ble.echo(false);

    Serial.println("Requesting Bluefruit info:");
       /* Print Bluefruit information */
    ble.info();

    Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
    Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
    Serial.println();

    ble.verbose(false);  // debug info is a little annoying after this point!

       /* Wait for connection */
    while (! ble.isConnected()) {
        if(countdown++ > 10) {
            break;
        }
        Serial.print(F("*"));
        delay(500);
    }

    Serial.println(F("****** CONNECT ************************"));

       // LED Activity command is only supported from 0.6.6
    if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
    {
           // Change Mode LED Activity
        Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
        ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
    }

       // Set Bluefruit to DATA mode
    Serial.println( F("Switching to DATA mode!") );
    ble.setMode(BLUEFRUIT_MODE_DATA);

    Serial.println(F("******** DATA ****************"));

}



