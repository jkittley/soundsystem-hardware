// **********************************************************************************
//
// This script is designed to test the RSSI - Requires a not to be sending messages
// periodically. The output can be plotted using the serial monitor.
//
// **********************************************************************************

#include <RFM69.h>              // https://www.github.com/lowpowerlab/rfm69
#include <I2S.h>

// **********************************************************************************

#define NODEID        1   // This device's ID
#define BASEID        100 // Basestation (PI node)
#define RELAYID       101 // The ID of the RFM69 to BLE Relay node
#define NETWORKID     20

#define RADIO_POWER   31   // power output ranges from 0 (5dBm) to 31 (20dBm)

// Are you using the RFM69 Wing? Uncomment if you are.
//#define USING_RFM69_WING

// The transmision frequency of the baord.
#define FREQUENCY      RF69_433MHZ

// Uncomment if this board is the RFM69HW/HCW not the RFM69W/CW
#define IS_RFM69HW_HCW

// Serial board rate - just used to print debug messages
#define SERIAL_BAUD   115200

// Battery pin
#define VBATPIN A7

bool DEBUG = true;  // Show debug messages

// How long to stay in config mode after the last acknowlegement
unsigned long stayInConfig = 1000 * 60; // 60 secs

// How long to wait for the relay node to respond to choose me requests befor returning to normal mode
unsigned long waitForChooseMeAck = 1000 * 10; // 10 secs

// Maximum time to spend in config mode - will exit config after x even if connected to Relay
unsigned long max_time_in_config = 1000 * 60 * 2; // 2 minutes

// Send interval
int send_interval_normal = 4000; // ms in normal mode
int send_interval_config = 1000; // ms in config mode

// LED Pins
int redPin = 5;
int greenPin = 10;
int bluePin = 11;

// Config button Pin
int configButton = A3;

int retries = 2;
int ackwait = 200;

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
#ifdef ENABLE_ATC
RFM69_ATC radio(RF69_SPI_CS, RF69_IRQ_PIN, false, RF69_IRQ_NUM);
#else
RFM69 radio(RF69_SPI_CS, RF69_IRQ_PIN, true, RF69_IRQ_NUM);
#endif

// Audio
#define ADC_SOUND_REF 65
#define DB_SOUND_REF 41
#define num_samples 256
int sample_index = 0;
int samples[num_samples];
int failure_count = 0;

// Payloads
typedef struct {
  uint8_t volume; // Volume
  uint8_t battery; // Battery voltage
  uint8_t reply; // Request Reply i.e. we are in config mode
} TXPayload;
TXPayload sendPayload;

typedef struct {
  uint8_t volume; // Volume
  uint8_t battery; // Battery voltage
  uint8_t rssi;    // rssi
} RXPayload;
RXPayload receivePayload;

typedef struct {
  uint8_t value;  // Volume
} ChooseMePayload;
ChooseMePayload chooseMePayload;

// Timers
unsigned long config_timer = 0;
unsigned long config_started = 0;
unsigned long choose_timer = 0;
unsigned long last_sent = 0;

// Mode states
bool is_in_config = false;
bool config_ack = false;
bool exit_after_next_msg = false;
bool enter_after_next_msg = false;
bool buttonEnabled = true;

//======================================================================================================
// Setup
//======================================================================================================

void setup() {

  //  Wait for Serial if we are in debug
  if (DEBUG) {
    setColor(255,255,255);
    unsigned long serial_timeout = millis() + 10000;
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < serial_timeout) {  
      delay(100); 
    } 
    setColor(0,0,0);
  }
  
  // Initialize the radio
  radio.initialize(FREQUENCY, NODEID, NETWORKID);
  #ifdef IS_RFM69HW_HCW
    radio.setHighPower(); //must include this only for RFM69HW/HCW!
  #endif
  radio.setPowerLevel(RADIO_POWER);

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
    radio.sleep();
    
    // Show color
    if (!is_in_config) setColor(0,255,0); // Green - let go of button
    if (is_in_config) setColor(255,0,0);  // Red - Let go of button
    // Wait for button release
    while(true) { if (digitalRead(configButton) == LOW) break; }
    setColor(0,0,0);
    // Act
    if (is_in_config) {
      if (Serial) Serial.println("Exiting config mode - Button pressed");
      endConfigMode();
    } else {
      if (Serial) Serial.println("Starting config mode - Button pressed");
      startConfigMode();
    }
  }

  // Listen for incoming messages
  if (radio.receiveDone()) {

    // If the message is from the base station and we are expecting them i.e. we are in config mode
    if (is_in_config && config_ack && radio.SENDERID == BASEID) {
//      if (Serial) Serial.println("Message from basestation");
      
      if (radio.DATALEN == sizeof(RXPayload)) {
//        if (Serial) Serial.println("Forwarding it to Relay.");
      
        receivePayload = *(RXPayload*)radio.DATA;        
//        if (Serial)  Serial.println(receivePayload.volume);
       
        if (radio.sendWithRetry(RELAYID, (const uint8_t*) &receivePayload, sizeof(receivePayload), retries, ackwait)) {
          if (Serial)  Serial.println("ACK recieved (For TX to Relay)");
          // Sustain config mode because relay replied
          config_timer = millis() + stayInConfig;
        } else {
          if (Serial)  Serial.println("NO - ACK recieved (For TX to Relay)");
        }
        
      }
    }    

    // Cant send ACKS after forwarding as globals overwritten and not befor for same reason.
    // ACK any incoming message
//    if (radio.ACKRequested()) {
//       radio.sendACK();
//       Serial.println(" - ACK sent.");
//    } 
    //delay(20);
    
  }

  // Exit config mode if we have not had any replies from the relay node in the given period
  if (!exit_after_next_msg && is_in_config && config_ack && config_timer < millis()) {
    if (Serial) Serial.println("Exiting config mode - No ACKs from relay / No packets from server to forward");
    endConfigMode();
  }

  // Exit config if time out reached - even if still connected
  if (!exit_after_next_msg && is_in_config && config_ack && config_started + max_time_in_config < millis()) {
    if (Serial) Serial.println("Exiting config mode - Time limit reached");
    endConfigMode();
  }


  // If the microphone has failed to get a reading x many times, reboot the node
  if (failure_count > 10000) {
    if (Serial) Serial.print("Resetting Microphone...");
    NVIC_SystemReset();
    failure_count = 0;
  }

  
  // Control send interval without using a delay - A delay in config mode prevents messages being passed from base to relay node
  if ((!is_in_config && millis() > last_sent + send_interval_normal) || (is_in_config && millis() > last_sent + send_interval_config)) {
    
    // We dont have enough samples yet
    if (sample_index < num_samples) {

      int sample = I2S.read();
      if (sample != 0 && sample != -1) {
        // convert to 18 bit signed
        sample >>= 14;
        samples[sample_index] = sample;
        sample_index++;
      } else {
        failure_count++;
      }

    // We do have enough samples so send a reading
    } else {
  
      if (Serial) Serial.println("Sending");
  
      float meanval = 0;
      float minsample = 100000;
      float maxsample = -100000;
      for (int i = 0; i < num_samples; i++) { meanval += samples[i]; }
      meanval /= num_samples;
  
      // Subtract it from all sapmles to get a 'normalized' output
      for (int i = 0; i < num_samples; i++) {  samples[i] -= meanval; }
  
      // Find the 'peak to peak' max
      maxsample, minsample;
      for (int i = 0; i < num_samples; i++) {
        minsample = min(minsample, samples[i]);
        maxsample = max(maxsample, samples[i]);
      }

      // Calc dB
      int dB = 20 * log10((float)maxsample / (float)ADC_SOUND_REF) + DB_SOUND_REF;
  
      // Reset sampling
      sample_index = 0;
      failure_count = 0;
  
      // Send data to relay node
      sendPayload.reply   = is_in_config ? 1 : 0; // If in config mode, request a reply with RSSI included
      sendPayload.battery = getBatteryLevel();
      sendPayload.volume  = dB;
      
      if (Serial) {
        Serial.print("Vol dB: "); Serial.print(sendPayload.volume);
        Serial.print(" | Bat: "); Serial.print(sendPayload.battery);
        Serial.print(" | Reply: "); Serial.println(sendPayload.reply);
      }

      // Send the message
      if (radio.sendWithRetry(BASEID, (const uint8_t*) &sendPayload, sizeof(sendPayload), retries, ackwait)) {
        if (Serial)  Serial.println("ACK recieved (For TX to Base)");
      } else {
        if (Serial)  Serial.println("NO ACK recieved (For TX to Base)");
      }

      // Record when message sent
      last_sent = millis();

      // If exit config mode has been requested then act on it now
      if (exit_after_next_msg) {
        actuallyExitConfigMode();
      } else if (enter_after_next_msg) {
        actuallyStartConfigMode();
      }
      
    }
  }
}

//===================================================
// Mode Control
//===================================================

void startConfigMode() {
  enter_after_next_msg = true;
}

void actuallyStartConfigMode() {
  enter_after_next_msg = false;
  exit_after_next_msg = false;
  
  config_timer = millis() + stayInConfig * 5;
  choose_timer = millis() + waitForChooseMeAck;
  config_started = millis();
  is_in_config = true;
  config_ack = false;
  
  // Ask relay if we can be chosen and wait for confirmation / error
  while (!config_ack) {
    setColor(0, 0, 255);
    if (Serial) Serial.println("Sending CHOOSEME");
    chooseMePayload.value = 10;

    radio.sleep();
    
    if (radio.sendWithRetry(RELAYID, (const uint8_t*) &chooseMePayload, sizeof(chooseMePayload), retries, ackwait)) {
      // If we get accepted
      if (Serial) Serial.println("Got ACK for CHOOSEME");
      setColor(0, 0, 255);
      config_ack = true;
      buttonEnabled = true;
      return;
    }
   
    // If we try for too long
    if (choose_timer < millis()) {
      endConfigMode();
      return;
    }
    // Blink LED and delay befor next send
    for (int x = 5; x >= 0; x--) {
      setColor(0, 0, 0);
      delay(250);
      setColor(0, 0, 255);
      delay(250);
    }
  }

}

void endConfigMode() {
  exit_after_next_msg = true;
}

void actuallyExitConfigMode() {
  exit_after_next_msg = false;
  enter_after_next_msg = false;
  is_in_config = false;
  config_ack = false;
  config_timer = 0;
  choose_timer = 0;
  setColor(0, 0, 0);
  buttonEnabled = true;
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
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);
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
  return measuredvbat*10;
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
  sprintf(buff, "\nListening at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(buff);
}

