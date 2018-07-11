
// **********************************************************************************
//
// RFM69 Radio sensor node.
//
// **********************************************************************************

#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>
#include <ArduinoJson.h>

bool DEBUG = true;  // Show debug messages

// Node and network config
#define NODEID        100   // The ID of this node)

// Are you using the RFM69 Wing? Uncomment if you are.
// #define USING_RFM69_WING

// The transmision frequency of the baord.
#define FREQUENCY      433.0

// Uncomment if this board is the RFM69HW/HCW not the RFM69W/CW
#define IS_RFM69HW_HCW true

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
// Dont put this on the stack:
uint8_t radioBuffer[RH_RF69_MAX_MESSAGE_LEN];

//===================================================
// Setup
//===================================================

void setup() {

  Serial.begin(SERIAL_BAUD);
  while (!Serial) {
    delay(100);
  }

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
  
  // Debug
  if (DEBUG) printDebugInfo();

}

//======================================================================================================
// Main loop
//======================================================================================================


void loop() {

   if (rf69_manager.available()) {
    
      uint8_t len = sizeof(radioBuffer);
      uint8_t from;
      if (rf69_manager.recvfromAck(radioBuffer, &len, &from)) {
        if (DEBUG) Serial.print("# Got packet from #"); Serial.println(from);
        //Serial.print(rf69.lastRssi());
        
        // Is it valid JSON    
        StaticJsonBuffer<200> jsonBuffer;
        JsonObject& root = jsonBuffer.parseObject((char*)radioBuffer);
        if (root.success()) {
          root.printTo(Serial);
          Serial.println("");

          // Send Ack - Only for valid messages
          char radiopacket[RH_RF69_MAX_MESSAGE_LEN];
          root["r"] = abs(rf69.lastRssi());
          root.printTo(radiopacket);
          
          if (!rf69_manager.sendtoWait((uint8_t *)radiopacket, strlen(radiopacket), from)) {
            Serial.println("Sending failed (no ack)");
          } 
          
        } else {
          Serial.println("Invalid JSON:");
          Serial.println((char*)radioBuffer); 
          
        }
      }
   }
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
  if (!Serial) return;
  Serial.println("-------------------------------------");
  Serial.println("------------ SERVER NODE ------------");
  Serial.println("-------------------------------------");
  Serial.print("I am node: "); Serial.println(NODEID);
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
  //sprintf(buff, "\nListening at %d Mhz...", FREQUENCY);
  Serial.println(buff);
}

