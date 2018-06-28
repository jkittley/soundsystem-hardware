
// **********************************************************************************
// 
// RFM69 Radio sensor node.
//                                                       
// **********************************************************************************

#include <RFM69.h>              // https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>          // https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>                // Included with Arduino IDE

// Node and network config
#define NODEID        254   // The ID of this node (must be different for every node on network)
#define NETWORKID     20   // The network ID

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

//===================================================
// Setup
//===================================================

void setup() {

  // Detect serial port coinnection and enable debug
  Serial.begin(SERIAL_BAUD);
  while (!Serial) { delay(100); }
    
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


  radio.promiscuous(true);

  // Debug
  printDebugInfo();

  Serial.println("------------------------------------");
  Serial.println("--------      SNOOPER      ---------");
  Serial.println("------------------------------------");

}

//===================================================
// Main loop
//===================================================

void loop() {
   
   if (radio.receiveDone()) {
    
      Serial.print("TO: "); Serial.println(radio.TARGETID);
      Serial.print("FROM: "); Serial.println(radio.SENDERID);
      Serial.print("DATA LEN: "); Serial.println(radio.DATALEN);
//      Serial.print("DATA: "); Serial.println(radio.DATA);
       Serial.println("-------------------------------------"); 
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

