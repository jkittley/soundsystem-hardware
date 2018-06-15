
// **********************************************************************************
// 
// RFM69 Radio sensor node.
//                                                       
// **********************************************************************************

#include <RFM69.h>              // https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>          // https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>                // Included with Arduino IDE
#include <ArduinoJson.h>        // https://arduinojson.org/d
#include <Adafruit_SleepyDog.h> // https://github.com/adafruit/Adafruit_SleepyDog
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"

bool LOG = false;    // Log data out
bool DEBUG = true;  // Show debug messages

// Node and network config
#define NODEID        21    // The ID of this node (must be different for every node on network)
#define NETWORKID     50   // The network ID

// Are you using the RFM69 Wing? Uncomment if you are.
#define USING_RFM69_WING 

// The transmision frequency of the baord. Change as needed.
#define FREQUENCY      RF69_433MHZ //RF69_868MHZ // RF69_915MHZ

// Uncomment if this board is the RFM69HW/HCW not the RFM69W/CW
#define IS_RFM69HW_HCW

// Serial board rate - just used to print debug messages
#define SERIAL_BAUD   115200

// **********************************************************************************
// **********************************************************************************
//
// END OF SETTINGS - DO not edit below this line
//
// **********************************************************************************
// **********************************************************************************

// Board and radio specific config - You should not need to edit
#if defined (__AVR_ATmega32U4__) && defined (USING_RFM69_WING)
    #define RF69_SPI_CS  10   
    #define RF69_RESET   11   
    #define RF69_IRQ_PIN 2 
    #define RF69_IRQ_NUM digitalPinToInterrupt(RF69_IRQ_PIN) 
#elif defined (__AVR_ATmega32U4__)
    #define RF69_RESET    4
    #define RF69_SPI_CS   8
    #define RF69_IRQ_PIN  7
    #define RF69_IRQ_NUM  4
#elif defined(ARDUINO_SAMD_FEATHER_M0) && defined (USING_RFM69_WING)
    #define RF69_RESET    11
    #define RF69_SPI_CS   10
    #define RF69_IRQ_PIN  6
    #define RF69_IRQ_NUM  digitalPinToInterrupt(RF69_IRQ_PIN)
#elif defined(ARDUINO_SAMD_FEATHER_M0)
    #define RF69_RESET    4
    #define RF69_SPI_CS   8
    #define RF69_IRQ_PIN  3
    #define RF69_IRQ_NUM  3
#endif


RFM69 radio(RF69_SPI_CS, RF69_IRQ_PIN, false, RF69_IRQ_NUM);

#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

//===================================================
// Setup
//===================================================

void setup() {

  // Detect serial port coinnection and enable debug
  Serial.begin(SERIAL_BAUD);
  int start = millis();
  while (!Serial) { 
    int remianing = (10000 - (millis() - start)) / 1000;
    if (remianing < 1) {
      DEBUG=false;
      break;  
    }
    delay(100);
  }
  if (DEBUG) Serial.println("Serial Started");
  
  // Init BLE
  initBLE();
  
  // Initialize the radio
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
  #ifdef IS_RFM69HW_HCW
    radio.setHighPower(); //must include this only for RFM69HW/HCW!
  #endif
  #ifdef ENCRYPTKEY
    radio.encrypt(ENCRYPTKEY);
  #endif

   // Wait for BLE     
   waitForBLE();
 
}

//===================================================
// Main loop
//===================================================

void loop() {
    // Send to BLE
    if (ble.isConnected()) {
      sendPayloadToBLE(random(0,100), random(0,100));
      delay (500);
    } else {
      waitForBLE();
    }
}

void sendPayloadToBLE(int vol, int rssi) {  
  String s = "data=0,"+String(rssi)+","+String(vol)+",0,0";
  // Send input data to host via Bluefruit
  ble.print(s);
  if (DEBUG) Serial.println(s);
  delay(250);
}


//===================================================
// BLE
//===================================================

void initBLE() {
  if (DEBUG) Serial.print(F("Initialising the Bluefruit LE module: "));
  if (!ble.begin(VERBOSE_MODE) ) {
    if (DEBUG) Serial.println("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?");
  }
  if ( FACTORYRESET_ENABLE ) {
    if (DEBUG) Serial.println(F("Performing a factory reset: "));
    if (!ble.factoryReset()){
      if (DEBUG) Serial.println("Couldn't factory reset");
    }
  }
  /* Disable command echo from Bluefruit */
  ble.echo(false);
  //ble.info();
  ble.verbose(false);  
  //ble.sendCommandCheckOK(F("AT+GAPDEVNAME=SoundSystemRelay"));
}

void waitForBLE() {

  if (ble.isConnected()) return;
  
   /* Wait for connection */
  if (DEBUG) Serial.println("Waiting for BLE connection");
  while (!ble.isConnected()) {
      delay(50);
  }

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) ) {
    // Change Mode LED Activity
    if (DEBUG) Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set module to DATA mode
  if (DEBUG) Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  delay(200);
}

