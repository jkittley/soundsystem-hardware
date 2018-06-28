
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

bool DEBUG = true;  // Show debug messages

// Node and network config
#define NODEID        100   // The ID of this node)
#define NETWORKID     20   // The network ID

// Are you using the RFM69 Wing? Uncomment if you are.
// #define USING_RFM69_WING

// The transmision frequency of the baord.
#define FREQUENCY      RF69_433MHZ

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

// Define payloads
typedef struct {
  uint8_t volume;  // Volume
  uint8_t battery; // Battery voltage
  uint8_t reply; // Want Reply i.e. node is in config mode
} RXPayload;
RXPayload receivePayload;

typedef struct {
  uint8_t volume; // Volume
  uint8_t battery; // Battery voltage
  uint8_t rssi;    // rssi
} TXPayload;
TXPayload sendPayload;

//===================================================
// Setup
//===================================================

void setup() {

  Serial.begin(SERIAL_BAUD);
  while (!Serial) {
    delay(100);
  }

  if (DEBUG) Serial.println("Serial Started");

  // Reset the radio
  resetRadio();

  // Initialize the radio
  radio.initialize(FREQUENCY, NODEID, NETWORKID);
  #ifdef IS_RFM69HW_HCW
    radio.setHighPower(); //must include this only for RFM69HW/HCW!
  #endif

  // Debug
  if (DEBUG) printDebugInfo();

}

//======================================================================================================
// Main loop
//======================================================================================================

void loop() {

  if (radio.receiveDone()) {
    if (radio.DATALEN != sizeof(RXPayload)) {
      if (DEBUG) Serial.println("# Invalid payload received, not matching Payload struct. -- ");
    } else {
      receivePayload = *(RXPayload*)radio.DATA; //assume radio.DATA actually contains our struct and not something else

      if (radio.ACKRequested()) {
          radio.sendACK();
          Serial.println(" - ACK sent.");
      }
          
      // Send to BLE
      sendPayloadToSerial(radio.SENDERID, radio.RSSI);
      
      // Send back RSSI if requested
      if (receivePayload.reply == 1) {
        delay(200);
        if (DEBUG) Serial.print("# Sending back RSSI to node: "); Serial.println(radio.SENDERID);
        sendPayload.volume = receivePayload.volume;
        sendPayload.battery = receivePayload.battery;
        sendPayload.rssi = abs(radio.RSSI);
        // Cant use retry as the is an issue with forwarding and sending acks
        radio.send(radio.SENDERID, (const uint8_t*) &sendPayload, sizeof(sendPayload));
      }
    }
  }
}

//===================================================
// Send Payload To Serial
//===================================================

void sendPayloadToSerial(int sender, int rssi) {
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root["sender"] = sender;
  root["rssi"]   = rssi;
  root["pay_volume"]  = receivePayload.volume;
  root["pay_battery"] = receivePayload.battery;
  root["pay_reply"] = receivePayload.reply;
  root.printTo(Serial);
  Serial.println();
  delay(250);
}

//===================================================
// Reset Radio
//===================================================

// Reset the Radio
void resetRadio() {
  if (DEBUG) Serial.print("Resetting radio...");
  pinMode(RF69_RESET, OUTPUT);
  digitalWrite(RF69_RESET, HIGH);
  delay(20);
  digitalWrite(RF69_RESET, LOW);
  delay(500);
  if (DEBUG) Serial.println(" complete");
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
  sprintf(buff, "\nListening at %d Mhz...", FREQUENCY == RF69_433MHZ ? 433 : FREQUENCY == RF69_868MHZ ? 868 : 915);
  Serial.println(buff);
}

//===================================================
// Split String
//===================================================

String getValue(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;
  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

