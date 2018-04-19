
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

// Debug mode
bool DEBUG = true;

// Node and network config
#define NODEID        101  // The ID of this node (must be different for every node on network)
#define NETWORKID     100  // The network ID
#define GATEWAYID     1    // Where to send sensor data

// Are you using the RFM69 Wing? Uncomment if you are.
#define USING_RFM69_WING 

// The transmision frequency of the baord. Change as needed.
#define FREQUENCY      RF69_433MHZ //RF69_868MHZ // RF69_915MHZ

// An encryption key. Must be the same 16 characters on all nodes. Comment out to disable
#define ENCRYPTKEY    "sampleEncryptKey"

// Uncomment if this board is the RFM69HW/HCW not the RFM69W/CW
#define IS_RFM69HW_HCW

// Uncomment to enable auto transmission control - adjusts power to save battery
#define ENABLE_ATC

// Serial board rate - just used to print debug messages
#define SERIAL_BAUD   115200

// Which LED
#define LED LED_BUILTIN

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

// Battery pin
#if defined (__AVR_ATmega32U4__) 
  #define VBATPIN A9
#elif defined(ARDUINO_SAMD_FEATHER_M0)
  #define VBATPIN A7
#endif

// Create Radio
#ifdef ENABLE_ATC
    RFM69_ATC radio(RF69_SPI_CS, RF69_IRQ_PIN, false, RF69_IRQ_NUM);
#else
    RFM69 radio(RF69_SPI_CS, RF69_IRQ_PIN, false, RF69_IRQ_NUM);
#endif

//
#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


//===================================================
// Setup
//===================================================

void setup() {
  Serial.begin(SERIAL_BAUD);

  while (!Serial) {
   ;
  }

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

  // Listen for messages from gateway
  listenToRFM69();

  listenToBLE();
  
  // Sleep
  sleepTime();
}

//===================================================
// Sleep mode (If not in config mode)
//===================================================

void sleepTime() {
  if (DEBUG) Serial.println("Sleeping...");
  delay(1000);
}

//===================================================
// Listen for messages
//===================================================

// Define payload
typedef struct {
  uint8_t battery; // Battery voltage
  uint8_t volume;  // Volume
} Payload;
Payload payload;

// Listen
void listenToRFM69() { 

    if (radio.receiveDone()) {

      if (DEBUG) {
        Serial.print('[sender: '); Serial.print(radio.SENDERID, DEC); Serial.println("] ");
        Serial.print("Data length: "); Serial.println(radio.DATALEN);
        Serial.print("[RX_RSSI:"); Serial.print(radio.RSSI); Serial.println("]");
      }

      if (radio.DATALEN != sizeof(Payload)) {
        if (DEBUG) Serial.print("# Invalid update received, not matching Payload struct. -- ");
      } else {    
        payload = *(Payload*)radio.DATA; //assume radio.DATA actually contains our struct and not something else
        if (radio.ACKRequested()) { radio.sendACK(); }
      
        sendToPayloadBLE();
      }
    }
}

//===================================================
// Blink
//===================================================

void Blink(int delayms, int numberTimes) {
  pinMode(LED, OUTPUT);
  for (int x=0; x < numberTimes; x++) {
    digitalWrite(LED, HIGH);
    delay(delayms);
    digitalWrite(LED, LOW);
    delay(delayms);
  }
}

//===================================================
// Reset Radio
//===================================================

// Reset the Radio
void resetRadio() {
  Serial.print("Resetting radio...");
  pinMode(RF69_RESET, OUTPUT);
  digitalWrite(RF69_RESET, HIGH);
  delay(20);
  digitalWrite(RF69_RESET, LOW);
  delay(500);
  Serial.println(" complete");
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
// Sensor Battery Level
//===================================================

float getBatteryLevel() {
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  return 100 * measuredvbat;
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

void sendToPayloadBLE() {
    if (DEBUG) Serial.print("Forwarding to BLE");
    if (ble.isConnected()) {
      String s = "data="+String(payload.battery)+","+String(payload.volume)+",0";
      // Send input data to host via Bluefruit
      ble.print(s);
      if (DEBUG) Serial.println(s);
    } else {
      if (DEBUG) Serial.println("Waiting for connection");
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

  /* Wait for connection */
  if (DEBUG) Serial.println("Waiting foir BLE connection");
  while (!ble.isConnected()) {
      delay(500);
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
}

