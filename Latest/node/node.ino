// **********************************************************************************
//
// This script is designed to test the RSSI - Requires a not to be sending messages
// periodically. The output can be plotted using the serial monitor.
//
// **********************************************************************************

#include <RH_RF69.h>
#include <RHReliableDatagram.h>
#include <I2S.h>

// **********************************************************************************

#define NODEID        18  // This device's ID
#define BASEID        100 // Basestation (PI node)
#define RELAYID       101 // The ID of the RFM69 to BLE Relay node
#define NETWORKID     20

#define RADIO_POWER   31   // power output ranges from 0 (5dBm) to 31 (20dBm)

// Are you using the RFM69 Wing? Uncomment if you are.
//#define USING_RFM69_WING

// The transmision frequency of the baord.
#define FREQUENCY      433.0

// Uncomment if this board is the RFM69HW/HCW not the RFM69W/CW
#define IS_RFM69HW_HCW

// Serial board rate - just used to print debug messages
#define SERIAL_BAUD   115200

// Battery pin
#define VBATPIN A7

bool DEBUG = false;  // Show debug messages

// How long to stay in config mode after the last acknowlegement
unsigned long configNoRelayMaxTime = 1000 * 20; // 60 secs

// How long to wait for the relay node to respond to choose me requests befor returning to normal mode
unsigned long waitForChooseMeAck = 1000 * 10; // 10 secs

// Maximum time to spend in config mode - will exit config after x even if connected to Relay
unsigned long maxTimeInConfigMode = 1000 * 60 * 5; // 2 minutes

// Send interval
int sendIntervalNormalMode = 8000; // ms in normal mode
int sendIntervalConfigMode = 250;//125 // ms in config mode

// LED Pins
#define LED_PIN_R 5
#define LED_PIN_G 10
#define LED_PIN_B 11

// Config button Pin
int configButton = A3;

int retries = 2;
int ackwait = 200;
int lostSignalCount = 0;
int ledStrength = 500;
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

// Audio
#define ADC_SOUND_REF 65
#define DB_SOUND_REF 41
#define num_samples 256
int sampleIndex = 0;
int samples[num_samples];
int sampleFailureCount = 0;

// Payloads
typedef struct {
  uint8_t id = 10; 
  uint8_t volume = 0;   // Volume
  uint8_t battery = 1;  // Battery voltage
} TXPayload;
TXPayload sendPayload;

typedef struct {
  uint8_t id = 11;
  uint8_t volume = 2;  // Volume
  uint8_t battery = 3; // Battery voltage
  uint8_t rssi = 4;    // rssi
} RXPayload;
RXPayload receivePayload;

typedef struct {
  uint8_t id = 12;
  uint8_t value;  // Volume
} ChooseMePayload;
ChooseMePayload chooseMePayload;

// Timers
unsigned long configNoRelayTimeout = 0;
unsigned long configStartTime = 0;
unsigned long configChooseMeTimeout = 0;
unsigned long messageLastSent = 0;

// Mode states
bool configModeRequested = false;
bool configModeAccepted = false;
bool buttonEnabled = true;

uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
uint8_t data[] = "  OK";
//======================================================================================================
// Setup
//======================================================================================================

void setup() {

  if (DEBUG) {
    setColor(ledStrength, 0, 0);
    delay(500);
    setColor(0, ledStrength, 0);
    delay(500);
    setColor(0, 0, ledStrength);
    delay(500);
  }
  
  //  Wait 10s for Serial if we are in debug
  if (DEBUG) {
    setColor(ledStrength, ledStrength, ledStrength);
    unsigned long serial_timeout = millis() + 10000;
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < serial_timeout) {
      delay(100);
    }
    setColor(0, 0, 0);
  }
  
  if (!rf69_manager.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(FREQUENCY)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
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
    
    // Stops interupts causing a crash - I think
    //radio.sleep();

    // Show color
    if (!configModeRequested) setColor(0, 255, 0); // Green - let go of button
    if (configModeRequested) setColor(255, 0, 0); // Red - Let go of button
    
    // Wait for button release
    while (true) { if (digitalRead(configButton) == LOW) break; }
    setColor(0, 0, 0);
    
    // Act
    if (configModeRequested) {
      if (Serial) Serial.println("Exiting config mode - Button pressed");
      endConfigMode(0);
    } else {
      if (Serial) Serial.println("Starting config mode - Button pressed");
      startConfigMode();
    }
  }

  // Exit config mode if we have not had any replies from the relay node in the given period
  if (inConfigMode() && configNoRelayTimeout < millis()) {
    if (Serial) Serial.println("Exiting config mode - No ACKs from relay / No packets from server to forward");
    endConfigMode(1);
  }

  // Exit config if time out reached - even if still connected
  if (inConfigMode() && configStartTime + maxTimeInConfigMode < millis()) {
    if (Serial) Serial.println("Exiting config mode - Time limit reached");
    endConfigMode(1);
  }

  // If the microphone has failed to get a reading x many times, reboot the node
  if (sampleFailureCount > 10000) {
    if (Serial) Serial.println("Resetting Microphone...");
    NVIC_SystemReset();
    sampleFailureCount = 0;
  }

  // Control send interval without using a delay - A delay in config mode prevents messages being passed from base to relay node
  if (isTimeToSend()) {
    // If sampelling is complete
    int dB = getSample();

    if (dB > 0) {

      float batt = getBatteryLevel();
      
      // Send data to relay node
      sendPayload.battery = batt;
      sendPayload.volume  = dB;
  
      if (Serial) { 
        Serial.println(inConfigMode());   
        Serial.print("Sending");
        Serial.print(" | Vol dB: "); Serial.print(sendPayload.volume);
        Serial.print(" | Bat: "); Serial.println(sendPayload.battery);
      }

     if (rf69_manager.sendtoWait((uint8_t *)&sendPayload, sizeof(sendPayload), BASEID)) {
      
         // Now wait for a reply from the server
        uint8_t len = sizeof(buf);
        uint8_t from;   
        if (rf69_manager.recvfromAckTimeout(buf, &len, ackwait, &from)) {
          buf[len] = 0; // zero out remaining string
           if (Serial)  Serial.println("--- ACK recieved (For TX to Base)");
          //
          // ACK RECEIVED FROM BASE STATION
          //
          
          // Forward this data to the BLE node
          if (inConfigMode() && buf[0] == receivePayload.id) {

              if (Serial) Serial.println("Forwarding Message to Relay Node");
              receivePayload = *(RXPayload*)buf;
              forwardToRelayNode();
          }
        } 
      } else {
          if (Serial)  Serial.println("--- NO ACK recieved (For TX to Base)");
          lostSignalCount++;
          //
          // NO RESPONSE RECEIVED FROM BASE STATION
          //
          if (lostSignalCount > 2) { // only send the data if no ack three times
            if (Serial) Serial.println("No signal from base three times"); ///!! fix

            receivePayload.volume = dB;
            receivePayload.rssi = 90;
            receivePayload.battery = batt;
            lostSignalCount = 0;
          }
          else {
             receivePayload.volume = dB;
          }
          
          // Forward this data to the BLE node
          if (inConfigMode()) { 

              if (Serial) Serial.println("Creating message to send to Relay Node");
              forwardToRelayNode();
          }
        }
        // Record when message sent
        messageLastSent = millis();
      }
  }
}

void forwardToRelayNode() {

  if (Serial) Serial.println("Data to forward");
  if (Serial) Serial.println(receivePayload.volume);
  if (Serial) Serial.println(receivePayload.rssi);
  if (Serial) Serial.println(receivePayload.battery);

  if (rf69_manager.sendtoWait((uint8_t *)&receivePayload, sizeof(receivePayload), RELAYID)){
     uint8_t len = sizeof(buf);
    uint8_t from;   
    if (rf69_manager.recvfromAckTimeout(buf, &len, ackwait, &from)) {
      if (Serial)  Serial.println("ACK recieved (For TX to Relay)");
      
      // Response from relay
      if (buf[0] == chooseMePayload.id) { 
            chooseMePayload = *(ChooseMePayload*)buf;

            // Thanks
            if (chooseMePayload.value == 200) {
              // Sustain config mode because relay replied
              if (Serial) { Serial.println("Relay Accepted Message"); }
              configNoRelayTimeout = millis() + configNoRelayMaxTime;

            // No thanks - not listening to you
            } else if (chooseMePayload.value < 200) {
              configNoRelayTimeout = 0;
              if (Serial) { Serial.print("Relay Rejected Message - "); Serial.println(chooseMePayload.value); }
            }
      } else {
        if (Serial)  Serial.println("Invalid Payload recieved (For TX to Relay)");
      }
    }
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
  
  if (rf69_manager.sendtoWait((uint8_t *)&chooseMePayload, sizeof(chooseMePayload), RELAYID)) {
    if (Serial) Serial.println("Sending request to relay");
    // Now wait for a reply from the server
    uint8_t len = sizeof(buf);
    uint8_t from;   
    if (rf69_manager.recvfromAckTimeout(buf, &len, ackwait, &from)) {
      buf[len] = 0; // zero out remaining string
      // Response from relay
      if (buf[0] == chooseMePayload.id) { 
        
          chooseMePayload = *(ChooseMePayload*)buf;

          // We got accepted
          if (chooseMePayload.value == 200) {
            if (Serial) Serial.println("CHOOSEME - Got ACK");
            setColor(0, 0, 255);
            configModeAccepted = true;
            buttonEnabled = true;
            return;
                // No thanks
          } else if (chooseMePayload.value < 200) {
            if (Serial) { Serial.print("CHOOSEME Rejected - "); Serial.println(chooseMePayload.value); }
            endConfigMode(chooseMePayload.value);
            return;
          }
      } else {
        if (Serial)  Serial.println("CHOOSEME - Invalid chooseMePayload response");
        endConfigMode(1);
      }
      
    } else {
        if (Serial)  Serial.println("CHOOSEME - NO ACK recieved (For TX to Relay)");
        rf69.sleep();
        delay(50);
        endConfigMode(1);
    }    
  }
  else {
    if (Serial)  Serial.println("CHOOSEME - Send Failed  (For TX to Relay)");
    rf69.sleep();
    delay(50);
    endConfigMode(1);
  }
}

void endConfigMode(int code) {
  configModeRequested = false;
  configModeAccepted = false;
  configNoRelayTimeout = 0;
  configChooseMeTimeout = 0;
  if (Serial) { Serial.print("Exiting config mode - code: "); Serial.println(code);  }
  
  switch (code) {
    case 0:    // Button press to exit
      if (Serial) { Serial.println("Button Press - Connection to Relay terminated"); }
      setColor(0, 0, 0);
      break;
    case 1:    // Time out
      if (Serial) { Serial.println("Time Out - Connection to Relay failed"); }
      setColor(ledStrength, 0, 0);
      break;
    case 101:  // Relay I'm not listening to you
      if (Serial) { Serial.println("Not listening to you anymore - Connection to Relay terminated"); }
      setColor(ledStrength, 0, ledStrength);
      break;
    case 102:    // Relay I'm not connected to BLE
      if (Serial) { Serial.println("No BLE Tablet - Connection to Relay terminated"); }
      setColor(0, 0, ledStrength);
      break;
  }
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
  if (Serial) Serial.println("Getting battery voltage");
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  return measuredvbat * 10;
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
  sprintf(buff, "\nListening at %d Mhz...", FREQUENCY);
  Serial.println(buff);
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



