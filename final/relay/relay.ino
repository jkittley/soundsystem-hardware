
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

bool LOG = true;    // Log data out
bool DEBUG = true;  // Show debug messages

// Node and network config
#define NODEID        101   // The ID of this node (must be different for every node on network)
#define NETWORKID     20   // The network ID

// Are you using the RFM69 Wing? Uncomment if you are.
#define USING_RFM69_WING 

// The transmision frequency of the baord. Change as needed.
#define FREQUENCY      RF69_433MHZ //RF69_868MHZ // RF69_915MHZ

// Uncomment if this board is the RFM69HW/HCW not the RFM69W/CW
#define IS_RFM69HW_HCW

// Serial board rate - just used to print debug messages
#define SERIAL_BAUD   115200

// Battery pin
#define VBATPIN A7

// Conversion to percentage
float min_db = 35; // Background level
float max_db = 90; // Everything above this is 100%
float worst_sig = abs(-90);
float best_sig  = abs(-25); 
   
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

int listening_to_node = 0;

// Define payloads
typedef struct {
  uint8_t volume;  // Volume
  uint8_t battery; // Battery voltage
  uint8_t rssi;    // rssi
} Payload;
Payload payload;

typedef struct {
  uint8_t value;  // Volume
} ChooseMePayload;
ChooseMePayload chooseMePayload;


//===================================================
// Setup
//===================================================

void setup() {

  // Detect serial port coinnection and enable debug
 
  if (LOG || DEBUG) { 
    Serial.begin(SERIAL_BAUD);
    while (!Serial) { delay(100); }
  }
  
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
  
  // Reset the radio
  resetRadio();

  // Initialize the radio
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
  #ifdef IS_RFM69HW_HCW
    radio.setHighPower(); //must include this only for RFM69HW/HCW!
  #endif
  #ifdef ENCRYPTKEY
    radio.encrypt(ENCRYPTKEY);
  #endif

  // Debug
  if (DEBUG) printDebugInfo();

}

//===================================================
// Main loop
//===================================================

void loop() {
   
   if (radio.receiveDone()) {
    
      if (DEBUG) Serial.println("Message received");

      // Choose me
      if (radio.DATALEN == sizeof(ChooseMePayload)) {
         if (DEBUG) Serial.println("CHOOSE ME request received");
         listening_to_node = radio.SENDERID;
          if (DEBUG) Serial.println("Listening now to node");
          if (DEBUG) Serial.println(listening_to_node);
        if (radio.ACKRequested()) {
            radio.sendACK();
            Serial.println(" - ACK sent.");
          }
          
      // Actual data
      } else if (radio.DATALEN == sizeof(Payload)) {   

        // If no node set, then take it
        if (listening_to_node == 0) {
          listening_to_node = radio.SENDERID;
        }
         
        if (listening_to_node == radio.SENDERID) {
          payload = *(Payload*)radio.DATA; //assume radio.DATA actually contains our struct and not something else
         
          // Send to BLE
          sendPayloadToBLE();

          if (radio.ACKRequested()) {
            radio.sendACK();
            Serial.println(" - ACK sent.");
          }
      
        } else {
          if (DEBUG) Serial.print("Ignoring node");
          if (DEBUG) Serial.println(radio.SENDERID);
        }

      // Unknown struct
      } else {
        if (DEBUG) Serial.println("# Invalid payload received, not matching Payload struct. -- "); 
      }
    }
    
}

void sendPayloadToBLE() {  
    float sendDB = max(0, min(100, 100 * ( (payload.volume-min_db) / (max_db-min_db) ) ));
    float sendRSSI = 100 - max(0, min(100, 100 * ( (abs(payload.rssi) - best_sig) / (worst_sig-best_sig) ) ));
    float this_battery = getBatteryLevel();
    float node_battery = float(payload.battery) / 10.0;
    String s = "data="+String(node_battery)+","+String(sendRSSI)+","+String(sendDB)+","+String(this_battery)+",0";
    // Send input data to host via Bluefruit
    if (DEBUG) Serial.println(s);
    if (ble.isConnected()) {
      ble.print(s);
      if (DEBUG) Serial.println("Sent via BLE");
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
  delay(500);
  if (Serial) Serial.println(" complete");
}

//===================================================
// Debug Message
//===================================================

// Print Info
void printDebugInfo() {
  Serial.print("I am node: "); Serial.println(NODEID);
  Serial.print("on network: "); Serial.println(NETWORKID);
  Serial.println("I am a relay");
  
  #if defined (__AVR_ATmega32U4__)
    Serial.println("AVR ATmega 32U4");
  #else
    Serial.println("SAMD FEATHER M0");
  #endif
  #ifdef USING_RFM69_WING
    Serial.println("Using RFM69 Wing: YES");
  #else
    Serial.println("Using RFM69 Wing: NO");
  #endif
  Serial.print("RF69_SPI_CS: "); Serial.println(RF69_SPI_CS);
  Serial.print("RF69_IRQ_PIN: "); Serial.println(RF69_IRQ_PIN);
  Serial.print("RF69_IRQ_NUM: "); Serial.println(RF69_IRQ_NUM);
  #ifdef ENABLE_ATC
    Serial.println("RFM69 Auto Transmission Control: Enabled");
  #else
    Serial.println("RFM69 Auto Transmission Control: Disabled");
  #endif
  #ifdef ENCRYPTKEY
    Serial.println("Encryption: Enabled");
  #else
    Serial.println("Encryption: Disabled");
  #endif
  char buff[50];
  sprintf(buff, "\nListening at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(buff);
}

//===================================================
// Split String
//===================================================

String getValue(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;
  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }
  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

//===================================================
// BLE
//===================================================

// Define struct for update payload
typedef struct {
  char key[20]; // command
  uint8_t value; // Battery voltage
} Update;
Update upd;

void listenToBLE() {
  String key = "";
  bool msgToSend = false;
  while ( ble.available() ) {
    msgToSend = true;
    int c = ble.read();
    key.concat(char(c));
  }
  if (msgToSend) {
    if (DEBUG) { Serial.println(String(key)); }
    
    key.toCharArray(upd.key, sizeof(key));
    
    if (radio.sendWithRetry(3, (const void*) &upd, sizeof(upd), 3, 500)) {
      if (DEBUG) Serial.println(" Acknoledgment received!");
    } else {
      if (DEBUG) Serial.println(" No Acknoledgment after retries");
    }
  }
}


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

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) ) {
    // Change Mode LED Activity
    if (DEBUG) Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set module to DATA mode
  if (DEBUG) Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);
}


float getBatteryLevel() {
  if (DEBUG) Serial.println("Getting battery voltage");
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  return measuredvbat;
}
