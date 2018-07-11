
// **********************************************************************************
//
// RFM69 Radio sensor node.
//
// **********************************************************************************

#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>
#include <ArduinoJson.h>

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"

bool DEBUG = true;  // Show debug messages

// Node and network config
#define NODEID        101   // The ID of this node (must be different for every node on network)
#define NETWORKID     20   // The network ID

// Are you using the RFM69 Wing? Uncomment if you are.
#define USING_RFM69_WING

// The transmision frequency of the baord. Change as needed.
#define FREQUENCY      433.0 

// Uncomment if this board is the RFM69HW/HCW not the RFM69W/CW
#define IS_RFM69HW_HCW true

// Serial board rate - just used to print debug messages
#define SERIAL_BAUD   115200

// Battery pin
#define VBATPIN A7

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

// Singleton instance of the radio driver
RH_RF69 rf69(RF69_SPI_CS, RF69_IRQ_PIN);
// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, NODEID);
// Dont put this on the stack:
uint8_t radioBuffer[RH_RF69_MAX_MESSAGE_LEN];

#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

int listening_to_node = 0;
float this_battery = 0;

//===================================================
// Setup
//===================================================

void setup() {

  //  Wait for Serial if we are in debug
  if (DEBUG) {
    unsigned long serial_timeout = millis() + 10000;
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < serial_timeout) {
      delay(100);
    }
  }

  // Init BLE
  initBLE();

  // Reset the radio
  // resetRadio();

  // Initialize the radio
  if (!rf69_manager.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  if (!rf69.setFrequency(FREQUENCY)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  rf69.setTxPower(20, IS_RFM69HW_HCW);  // range from 14-20 for power, 2nd arg must be true for 69HCW
  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);  
  if (Serial) Serial.println("Radio Initialised");
  
  // Init battery
  this_battery = 0; //getBatteryLevel();

}

//===================================================
// Main loop
//===================================================

void loop() {
  
  if (rf69_manager.available()) {

    uint8_t len = sizeof(radioBuffer);
    uint8_t from;
    if (rf69_manager.recvfromAck(radioBuffer, &len, &from)) {
        if (Serial) { Serial.print("Message received from node: "); Serial.println(from); }

        // Is it valid JSON    
        StaticJsonBuffer<200> jsonBuffer;
        JsonObject& root = jsonBuffer.parseObject((char*)radioBuffer);
        if (root.success()) {
          processMessage(root, from);
        } else {
          Serial.println("Invalid JSON:");
          Serial.println((char*)radioBuffer); 
        }
    }
  }
}

void processMessage(JsonObject& json, int sender) {
  
  if (Serial) {
    json.printTo(Serial);
    Serial.println("");
  }

  StaticJsonBuffer<100> jsonBuffer;
  JsonObject& replyJson = jsonBuffer.createObject();
  replyJson["msg"] = "Unknown";

  // Process
  if (!ble.isConnected()) { 
    replyJson["msg"] = "BLE Not Connected";
    
  } else if (json["t"] == "chooseme") {
    listening_to_node = sender;
    replyJson["msg"] = "OK";
    
  } else if (json["t"] == "data" && sender != listening_to_node) {
    replyJson["msg"] = "Not Listening";
      
  } else if (json["t"] == "data" && sender == listening_to_node) {

     if (Serial) Serial.print("To BLE");
     json.printTo(Serial);
        
      // Send input data to host via Bluefruit       
      ble.print(json["d"].as<String>());
      delay(50);
      ble.print(json["b"].as<String>());
      replyJson["msg"] = "OK";
            
  } else {
     replyJson["msg"] = "Unknown Type";
  }
    
  // Send reply  
  if (Serial) {
    Serial.print("To Node: ");
    replyJson.prettyPrintTo(Serial);
    Serial.println("");
  }
  
  char radiopacket[RH_RF69_MAX_MESSAGE_LEN];
  replyJson.printTo(radiopacket);
  // Send a message
  if (rf69_manager.sendtoWait((uint8_t *)radiopacket, strlen(radiopacket), sender)) {

  } 
     
}


//===================================================
// Reset Radio
//===================================================

// Reset the Radio
void resetRadio() {
  if (Serial) Serial.print("Resetting radio...");
  pinMode(RF69_RESET, OUTPUT);
  digitalWrite(RF69_RESET, HIGH);
  delay(20);
  digitalWrite(RF69_RESET, LOW);
  delay(1000);
  if (Serial) Serial.println(" complete");
}

//===================================================
// BLE
//===================================================

void initBLE() {
  if (Serial) Serial.print(F("Initialising the Bluefruit LE module: "));
  if (!ble.begin(VERBOSE_MODE) ) {
    if (Serial) Serial.println("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?");
  }
  if ( FACTORYRESET_ENABLE ) {
    if (Serial) Serial.println(F("Performing a factory reset: "));
    if (!ble.factoryReset()) {
      if (Serial) Serial.println("Couldn't factory reset");
    }
  }
  /* Disable command echo from Bluefruit */
  ble.echo(false);
  //ble.info();
  ble.verbose(false);

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) ) {
    // Change Mode LED Activity
    if (Serial) Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set module to DATA mode
  if (Serial) Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);
}

//===================================================
// Battery
//===================================================

//float getBatteryLevel() {
//  //  if (Serial) Serial.println("Getting battery voltage");
//  float measuredvbat = analogRead(VBATPIN);
//  measuredvbat *= 2;    // we divided by 2, so multiply back
//  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
//  measuredvbat /= 1024; // convert to voltage
//  return measuredvbat;
//}
