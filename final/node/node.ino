// **********************************************************************************
//
// This script is designed to test the RSSI - Requires a not to be sending messages
// periodically. The output can be plotted using the serial monitor.
//
// **********************************************************************************

#include <RHReliableDatagram.h>
#include <RH_RF69.h>
#include <SPI.h>
#include <I2S.h>
#include <ArduinoJson.h>

// **********************************************************************************

#define NODEID        13   // This device's ID
#define BASEID        100 // Basestation (PI node)
#define RELAYID       101 // The ID of the RFM69 to BLE Relay node
#define NETWORKID     20

#define RADIO_POWER   31   // power output ranges from 0 (5dBm) to 31 (20dBm)

// Are you using the RFM69 Wing? Uncomment if you are.
//#define USING_RFM69_WING

// The transmision frequency of the baord.
#define FREQUENCY      433.0

// Uncomment if this board is the RFM69HW/HCW not the RFM69W/CW
#define IS_RFM69HW_HCW true

// Serial board rate - just used to print debug messages
#define SERIAL_BAUD   115200

// Battery pin
#define VBATPIN A7

bool DEBUG = true;  // Show debug messages

// How long to stay in config mode after the last acknowlegement
unsigned long configNoRelayMaxTime = 1000 * 20; // 60 secs

// How long to wait for the relay node to respond to choose me requests befor returning to normal mode
unsigned long waitForChooseMeAck = 1000 * 10; // 10 secs

// Maximum time to spend in config mode - will exit config after x even if connected to Relay
unsigned long maxTimeInConfigMode = 1000 * 60 * 5; // 2 minutes

// Send interval
int sendIntervalNormalMode = 8000; // ms in normal mode
int sendIntervalConfigMode = 500; // ms in config mode

// LED Pins
#define LED_PIN_R 5
#define LED_PIN_G 10
#define LED_PIN_B 11

// Config button Pin
int configButton = A3;

int retries = 2;
int ackwait = 300;

// Conversion to percentage
float min_db = 35; // Background level
float max_db = 90; // Everything above this is 100%
float default_sig = abs(-91);
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

// Create Radio
RH_RF69 rf69(RF69_SPI_CS, RF69_IRQ_PIN);
// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, NODEID);
uint8_t radioBuffer[RH_RF69_MAX_MESSAGE_LEN];

// Audio
#define ADC_SOUND_REF 65
#define DB_SOUND_REF 41
#define num_samples 256
int sampleIndex = 0;
int samples[num_samples];
int sampleFailureCount = 0;

// Timers
unsigned long configNoRelayTimeout = 0;
unsigned long configStartTime = 0;
unsigned long configChooseMeTimeout = 0;
unsigned long messageLastSent = 0;

// Mode states
bool configModeRequested = false;
bool configModeAccepted = false;
bool buttonEnabled = true;

//======================================================================================================
// Setup
//======================================================================================================

void setup() {

  if (DEBUG) {
    setColor(255, 0, 0);
    delay(500);
    setColor(0, 255, 0);
    delay(500);
    setColor(0, 0, 255);
    delay(500);
  }

  //  Wait for Serial if we are in debug
  if (DEBUG) {
    setColor(255, 255, 255);
    unsigned long serial_timeout = millis() + 10000;
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < serial_timeout) {
      delay(100);
    }
    setColor(0, 0, 0);
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
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08
                  };
  rf69.setEncryptionKey(key);

  // Init Microphone
  if (!I2S.begin(I2S_PHILIPS_MODE, 16000, 32)) {
    if (Serial) Serial.println("Failed to initialize I2S!");
    while (1); // do nothing
  }

  // Button
  pinMode(configButton, OUTPUT);

  //
  if (Serial) printDebugInfo();
}

//======================================================================================================
// Main loop
//======================================================================================================



void loop() {

  // If the button is pressed then try to enter config mode
  if (digitalRead(configButton) == HIGH && buttonEnabled) {

    // Disable buttong until tasks are done - stops debounce issues
    buttonEnabled = false;
    if (Serial) Serial.println("Switching mode");

    // Show color
    if (!configModeRequested) setColor(0, 255, 0); // Green - let go of button
    if (configModeRequested) setColor(255, 0, 0); // Red - Let go of button

    // Wait for button release
    while (true) {
      if (digitalRead(configButton) == LOW) break;
    }
    setColor(0, 0, 0);

    // Act
    if (configModeRequested) {
      endConfigMode("Exiting config mode - Button pressed");
    } else {
      if (Serial) Serial.println("Starting config mode - Button pressed");
      startConfigMode();
    }
  }

  // Exit config mode if we have not had any replies from the relay node in the given period
  if (inConfigMode() && configNoRelayTimeout < millis()) {
    endConfigMode("No ACK from relay / No packets from server to forward");
  }

  // Exit config if time out reached - even if still connected
  if (inConfigMode() && configStartTime + maxTimeInConfigMode < millis()) {
    if (Serial) Serial.println();
    endConfigMode("Time limit reached");
  }

  // If the microphone has failed to get a reading x many times, reboot the node
  if (sampleFailureCount > 10000) {
    if (Serial) Serial.println("Resetting device...");
    NVIC_SystemReset();
    sampleFailureCount = 0;
  }

  if (isTimeToSend()) {
    int dB = getSample();
    if (dB > 0) sendPacket(dB);
  }
}

//===================================================
// Send Packet to Base
//===================================================

void sendPacket(int dB) {

  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& txJson = jsonBuffer.createObject();
  txJson["t"] = "data";
  txJson["r"] = default_sig;
  txJson["b"] = getBatteryLevel();
  txJson["v"] = dB;
 
  if (Serial) {
    Serial.print("JSON: ");
    txJson.printTo(Serial);
    Serial.println("");
  }
  char radiopacket[RH_RF69_MAX_MESSAGE_LEN];
  txJson.printTo(radiopacket);

  if (Serial) Serial.print("BUFFER: "); Serial.println((char*)radiopacket); 

  // Send a message
  if (rf69_manager.sendtoWait((uint8_t *)radiopacket, strlen(radiopacket), BASEID)) {
    
    // Response
    uint8_t len = sizeof(radioBuffer);
    uint8_t from;   
    if (rf69_manager.recvfromAckTimeout(radioBuffer, &len, 2000, &from)) {

      if (Serial) Serial.println("Message Sent");

      // Now when we see a reply forward the packet with the RSSI to the relay node
      if (inConfigMode()) {
        radioBuffer[len] = 0; // zero out remaining string
        StaticJsonBuffer<200> jsonBuffer;
        JsonObject& rxJson = jsonBuffer.parseObject((char*)radioBuffer);
        forwardToRelay(rxJson);
      }
    
      // Record when message was acknowledged
      messageLastSent = millis();
     
        
    } else {
      if (Serial) Serial.println("No reply, is anyone listening?");

      // Now when we DONT see a reply forward the orignal packet without an to the relay node
      if (inConfigMode()) {
        forwardToRelay(txJson);
      }
      
    }

  } else {
    if (Serial) Serial.println("Sending failed (no ack)");

    // Now when we DONT see a reply forward the orignal packet without an to the relay node
    if (inConfigMode()) {
      forwardToRelay(txJson);
    }
    
  }
}

//===================================================
// Forward Packet to Relay
//===================================================

void forwardToRelay(JsonObject& json) {

  if (Serial) Serial.print("Forwarding to Relay: ");
  if (Serial) json.printTo(Serial);
  if (Serial) Serial.println("");



  float sendDB = max(0, min(100, 100 * ( ( float(json["v"]) - min_db) / (max_db - min_db) ) ));
  float sendRSSI = 100 - max(0, min(100, 100 * ( ( float(json["r"]) - best_sig) / (worst_sig - best_sig) ) ));
  float node_battery = float(json["b"]);
  
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& relayJson = jsonBuffer.createObject();
  relayJson["t"] = "data";
  relayJson["d"] = "d=" + String(NODEID) + "," + String(sendRSSI) + "," + String(sendDB) + ",";
  relayJson["b"] = "b=" + String(NODEID) + "," + String(node_battery) + "," +  "0" + ",";
  char radiopacket[RH_RF69_MAX_MESSAGE_LEN];
  relayJson.printTo(radiopacket);

  // Send
  if (rf69_manager.sendtoWait((uint8_t *)radiopacket, strlen(radiopacket), RELAYID)) {
    
    // Response
    uint8_t len = sizeof(radioBuffer);
    uint8_t from;   
    if (rf69_manager.recvfromAckTimeout(radioBuffer, &len, 2000, &from)) {
      radioBuffer[len] = 0; // zero out remaining string

      StaticJsonBuffer<200> jsonBuffer;
      JsonObject& relayJson = jsonBuffer.parseObject((char*)radioBuffer);
      if (relayJson.success()) {

        if (Serial) Serial.print("RELAY REPLY: ");
        if (Serial) relayJson.printTo(Serial);
      
        if (relayJson["msg"] != "OK") {
          
          endConfigMode(relayJson["msg"]);
          configNoRelayTimeout = millis() + configNoRelayMaxTime;
       
        }

      } else {
        endConfigMode("Invalid JSON reply from relay");
      }
      
    } else {
      
      endConfigMode("No reply, is the relay listening?");
      
    }
    
  } else {
    
    endConfigMode("Failed to send to Relay");
    
  }
}




//===================================================
// Mode Control
//===================================================

void startConfigMode() {
  configNoRelayTimeout = millis() + configNoRelayMaxTime * 2;
  configChooseMeTimeout = millis() + waitForChooseMeAck;
  configStartTime = millis();
  configModeRequested = true;
  configModeAccepted = false;

  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& txJson = jsonBuffer.createObject();
  txJson["t"] = "chooseme";
  if (Serial) {
    Serial.print("To Relay: ");
    txJson.prettyPrintTo(Serial);
    Serial.println("");
  }
  
  char radiopacket[RH_RF69_MAX_MESSAGE_LEN];
  txJson.printTo(radiopacket);

  // Send a message
  if (rf69_manager.sendtoWait((uint8_t *)radiopacket, strlen(radiopacket), RELAYID)) {

    // Response
    uint8_t len = sizeof(radioBuffer);
    uint8_t from;   
    if (rf69_manager.recvfromAckTimeout(radioBuffer, &len, 2000, &from)) {
      radioBuffer[len] = 0; // zero out remaining string

      StaticJsonBuffer<200> jsonBuffer;
      JsonObject& relayReplyJson = jsonBuffer.parseObject((char*)radioBuffer);
      if (relayReplyJson.success()) {

        if (Serial) {
          relayReplyJson.prettyPrintTo(Serial);
          Serial.println("");
        }
        
        if (relayReplyJson["msg"] != "OK") {
          
          endConfigMode(relayReplyJson["msg"]);
          
        } else {
          
          if (Serial) Serial.println("All OK");
          configModeAccepted = true;
          setColor(0, 0, 255);
          buttonEnabled = true;
          
        }
        
      } else {
        endConfigMode("Invalid JSON CHOOSEME reply from Relay");
      }
    
    } else {
      endConfigMode("No ACK from relay - CHOOSEME request");
    }
    
  } else {
    endConfigMode("Failed to send CHOOSEME to Relay");
  }
  
}




void endConfigMode(String reason) {

  configModeRequested = false;
  configModeAccepted = false;
  configNoRelayTimeout = 0;
  configChooseMeTimeout = 0;
  
  if (Serial) {
    Serial.print("Exiting config mode - reason: ");
    Serial.println(reason);
  }

  setColor(255, 0, 0);
  delay(500);
  buttonEnabled = true;
  setColor(0, 0, 0);
}

//===================================================
// Radio
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
// LED Color
//===================================================

void setColor(int red, int green, int blue) {
#ifdef COMMON_ANODE
  red = 255 - red;
  green = 255 - green;
  blue = 255 - blue;
#endif
  analogWrite(LED_PIN_R, red);
  analogWrite(LED_PIN_G, green);
  analogWrite(LED_PIN_B, blue);
}

//===================================================
// Battery
//===================================================

float getBatteryLevel() {
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  return measuredvbat;
}

//===================================================
// Debug Message
//===================================================

// Print Info
void printDebugInfo() {
  if (!Serial) return;
  Serial.println("-------------------------------------");
  Serial.println("------------ SENSOR NODE ------------");
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
  Serial.print("\nListening to ");
  Serial.print(FREQUENCY);
  Serial.println(" Mhz");
}


//===================================================
// Get Audio sample
//===================================================

int getSample() {
  // We dont have enough samples yet
  if (sampleIndex < num_samples) {

    int sample = I2S.read();
    if (sample != 0 && sample != -1) {
      // convert to 18 bit signed
      sample >>= 14;
      samples[sampleIndex] = sample;
      sampleIndex++;
    } else {
      sampleFailureCount++;
    }
    return 0;
  }

  float meanval = 0;
  float minsample = 100000;
  float maxsample = -100000;
  for (int i = 0; i < num_samples; i++) {
    meanval += samples[i];
  }
  meanval /= num_samples;

  // Subtract it from all sapmles to get a 'normalized' output
  for (int i = 0; i < num_samples; i++) {
    samples[i] -= meanval;
  }

  // Find the 'peak to peak' max
  maxsample, minsample;
  for (int i = 0; i < num_samples; i++) {
    minsample = min(minsample, samples[i]);
    maxsample = max(maxsample, samples[i]);
  }

  // Reset sampling
  sampleIndex = 0;
  sampleFailureCount = 0;

  // Calc dB
  return int(20 * log10((float)maxsample / (float)ADC_SOUND_REF) + DB_SOUND_REF);
}

//===================================================
// Helpers
//===================================================

bool inConfigMode() {
  return configModeRequested && configModeAccepted;
}

bool isTimeToSend() {
  if (!configModeRequested && millis() > messageLastSent + sendIntervalNormalMode) return true;
  if (configModeRequested && millis() > messageLastSent + sendIntervalConfigMode) return true;
  return false;
}



