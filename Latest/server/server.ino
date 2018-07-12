
// **********************************************************************************
//
// RFM69 Radio sensor node.
//
// **********************************************************************************

#include <RH_RF69.h>
#include <RHReliableDatagram.h>
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
#define FREQUENCY      433.0

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

// Singleton instance of the radio driver
RH_RF69 rf69(RF69_SPI_CS, RF69_IRQ_PIN);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, NODEID);

// Define payloads
typedef struct {
  uint8_t id = 10;
  uint8_t volume;  // Volume
  uint8_t battery; // Battery voltage
} RXPayload;
RXPayload receivePayload;

typedef struct {
  uint8_t id = 11;
  uint8_t volume; // Volume
  uint8_t battery; // Battery voltage
  uint8_t rssi;    // rssi
} TXPayload;
TXPayload sendPayload;


// Dont put this on the stack:
uint8_t data[] = "And hello back to you";
// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];

//===================================================
// Setup
//===================================================

void setup() {

  Serial.begin(SERIAL_BAUD);
  while (!Serial) {
    delay(100);
  }

  // Reset the radio
  resetRadio();
if (!rf69_manager.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }

  Serial.println("RFM69 radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(433)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
      

  // Debug
  if (DEBUG) printDebugInfo();

}

//======================================================================================================
// Main loop
//======================================================================================================

void loop() {
  if (rf69_manager.available())
  {
    // Wait for a message addressed to us from the client
    uint8_t len = sizeof(buf);
    uint8_t from;
   
    if (rf69_manager.recvfromAck(buf, &len, &from)) {
      buf[len] = 0; // zero out remaining string
      if (Serial) { Serial.print("Message received from node: "); Serial.println(from); }
    // Choose me
      if (buf[0] != receivePayload.id) {
      if (DEBUG) Serial.println("# Invalid payload received, not matching Payload struct. -- ");
    } else {
      receivePayload = *(RXPayload*)buf; //assume radio.DATA actually contains our struct and not something else

    
          if (DEBUG) Serial.println("- ACK sent with RSSI info");
          sendPayload.volume = receivePayload.volume;
          sendPayload.battery = receivePayload.battery;
          sendPayload.rssi = abs(rf69.lastRssi());
          // Cant use retry as the is an issue with forwarding and sending acks
          if (!rf69_manager.sendtoWait((uint8_t *)&sendPayload, sizeof(sendPayload), from)) {
          if (Serial) Serial.println("Sending failed (no ack)");}
        else {
          if (Serial) Serial.println("Sending success");
        }
        
      }

      // Send to BLE
      sendPayloadToSerial(from, rf69.lastRssi());
      
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
  root["pay_battery"] = float(receivePayload.battery) / 10.0;
  root.printTo(Serial);
  Serial.println();
  delay(250);
}

//===================================================
// Reset Radio
//===================================================

// Reset the Radio
void resetRadio() {
  if (Serial) Serial.print("Resetting radio...");
  pinMode(RF69_RESET, OUTPUT);
  digitalWrite(RF69_RESET, HIGH);
  delay(10);
  digitalWrite(RF69_RESET, LOW);
  delay(10);
  if (Serial) Serial.println(" complete");
}

//===================================================
// Debug Message
//===================================================

// Print Info
void printDebugInfo() {
  if (!Serial) return;
  Serial.println("-------------------------------------");
  Serial.println("------------ SERVER NODE ------------");
  Serial.println("-------------------------------------");
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
  sprintf(buff, "\nListening at %d Mhz...", FREQUENCY);
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

