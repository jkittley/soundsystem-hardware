// **********************************************************************************
// 
// RFM69 Radio sensor node.
//                        
// NOTES
// for this code to read out battery, change 
// Microphone PIN_I2S_SD to 12 in variant.h so 
// that we can use Pin A7. Full path is something like
// /arduino15/packages/adafruit/hardware/samd/1.0.21/
// variants/feather_m0/variant.h                               
// **********************************************************************************

// **********************************************************************************
// LED Patterns
//
// Off 100ms Green 100ms    = Waiting for Serial connection
// Constant Red             = System has been killed programmatically due to an error
// Blue with Red Flashes    = In config Mode and failed to receive ACK for message just sent
// Blue with Green Flashes  = In config Mode and received ACK for message just sent
// Blue with Yellow Flashes = In normal Mode and update message received
// Off with Red Flashes     = In normal Mode and failed to receive ACK for message just sent
// Off with Green Flashes   = In normal Mode and received ACK for message just sent
// Off with Yellow Flashes  = In normal Mode and update message received
//
// **********************************************************************************

#include <RFM69.h>              // https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>                // Included with Arduino IDE
#include <Adafruit_SleepyDog.h> // https://github.com/adafruit/Adafruit_SleepyDog
#include <I2S.h>                // Included with Arduino IDE

// **********************************************************************************

bool DEBUG = true; // Auto set to false if no serial connection.
bool USELED = true;

// Node and network config
#define NODEID        215    // The ID of this node (must be different for every node on network)
#define NETWORKID     100    // The network ID
#define GATEWAYID     1      // Where to send sensor data
#define CONFIGID      101    // Where to send config data
int TRANSMITPERIOD  = 10000; // Transmission interval in ms e.g. 5000 = every 5 seconds 
int CONFIGPERIOD    = 15000;  // Time can be in config mode without ack
int dest = GATEWAYID;

#define NUM_RETRYS    3    // How many times to retry sending a message
#define RETRY_WAIT    200  // How long to wait for Acknoledgement
#define RADIO_POWER   31   // power output ranges from 0 (5dBm) to 31 (20dBm)

// Are you using the RFM69 Wing? Uncomment if you are.
//#define USING_RFM69_WING 

// The transmision frequency of the baord. Change as needed.
#define FREQUENCY      RF69_433MHZ //RF69_868MHZ // RF69_915MHZ

// An encryption key. Must be the same 16 characters on all nodes. Comment out to disable
//#define ENCRYPTKEY    "ABCDEFGHIJKLMNOP"

// Uncomment if this board is the RFM69HW/HCW not the RFM69W/CW
#define IS_RFM69HW_HCW

// Uncomment to enable auto transmission control - adjusts power to save battery
// #define ENABLE_ATC

// Serial board rate - just used to print debug messages
#define SERIAL_BAUD   115200

// Which LED
#define LED LED_BUILTIN
int redPin = 5;
int greenPin = 10;
int bluePin = 11;

// Battery pin
#define VBATPIN A7

// Button pin
int modeButtonPin = A3;

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

// Operation modes
#define MODE_NORMAL 1
#define MODE_CONFIG  2
int mode = MODE_NORMAL;
unsigned long modeTimer = 0;

// Mic
#define SAMPLES 1024//2048
#define ADC_SOUND_REF 65
#define DB_SOUND_REF 41
int samples[SAMPLES];

int gatewayRSSI = 0;

//===================================================
// Setup
//===================================================

void setup() {
  
  pinMode(modeButtonPin, INPUT);
  
  if (USELED) {
    pinMode(redPin, OUTPUT);
    pinMode(greenPin, OUTPUT);
    pinMode(bluePin, OUTPUT);
  }

  // LED TEST Red... Green... Blue...
  if (USELED) {
    setColor(255,0,0);
    delay(1000);
    setColor(0,255,0);
    delay(1000);
    setColor(0,0,255);
    delay(1000);
  }
  
  // Auto Debug Mode - i.e. is there a serial connection then DEBUG = true
  Serial.begin(SERIAL_BAUD);
  int start = millis();
  while (!Serial) { 
    ledBlink(0,255,0, 100, 1);
    int remianing = (10000 - (millis() - start)) / 1000;
    if (remianing < 1) {
      DEBUG=false;
      break;  
    }
    delay(100);
  }
     
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
  
  // Set radio Power level  
  radio.setPowerLevel(RADIO_POWER); 
 
 // init
  if (!initMicrophone()) { 
    if (DEBUG) Serial.println("Failed to initialize I2S!"); 
    setColor(255,0,0);
    die();
  }
  
  //leave the voltage regulator and clock 
  // on while sleeping for the mic
  SYSCTRL->DFLLCTRL.bit.RUNSTDBY=1; 
  SYSCTRL->VREG.bit.RUNSTDBY=1;

  // Debug
  if (DEBUG) printDebugInfo();

  // Start in config mode
  setModeToConfig();
 
}

//===================================================
// Main loop
//===================================================

void loop() {

  // Send
  if (mode == MODE_NORMAL) {
    sendReading();
  } else {
    sendConfig();
  }

  // Listen for messages from gateway
  listenForMessages();
  
  // Revert to normal if no acks from alternative
  autoRevertModeManager();
  
  // Check for mode button press
  checkModeButton();
  
  // Sleep
  sleepTime();
  
}

//===================================================
// Sleep mode (If not in config mode)
//===================================================

void sleepTime() {
  if (mode == MODE_NORMAL) {
      radio.sleep();
      Watchdog.sleep(TRANSMITPERIOD);
  } else {
     myDelay(1000);
  }
}

//===================================================
// Display the mode of operation i.e. Normal or Debug
//===================================================

// Check if mode button has been pressed
void checkModeButton() {
   if (digitalRead(modeButtonPin) == HIGH) { ///
     setModeToConfig();
  } 
}

void setModeToConfig() {
   mode = MODE_CONFIG;
   dest = CONFIGID;
   modeTimer = millis(); 
   if (DEBUG) Serial.println("Mode button pressed");
}

void autoRevertModeManager() {
  if (mode == MODE_CONFIG) {
    if (millis() - modeTimer > CONFIGPERIOD) {
      if (DEBUG) Serial.println("Auto reverting as timed out");
      mode = MODE_NORMAL;
      dest = GATEWAYID;
    }
  }
}

//===================================================
// Listen form server messages after sending a msg
//===================================================

// Define struct for update payload
typedef struct {
  char key[20]; // command
  uint8_t value; // Battery voltage
} Update;
Update upd;

// 
void listenForMessages() { 
  unsigned long waitfor = 1500;
  if (mode == MODE_NORMAL) { waitfor = 2000; }
  
  unsigned long started = millis();  
  while (millis() < started + waitfor) {
    
    ledBlink(255,255,0, 100, 1);
    
    if (radio.receiveDone()) {
      
      if (DEBUG) {
        Serial.print("Sender: "); Serial.println(radio.SENDERID);
        Serial.print("Data length: "); Serial.println(radio.DATALEN);
        Serial.print("RX_RSSI:"); Serial.println(radio.RSSI);
        gatewayRSSI = radio.RSSI;
      }
  
//      if (radio.DATALEN != sizeof(Update)) {
//        if (DEBUG) Serial.print("# Invalid update received, not matching Upload struct. -- ");
//      } else {    
//        upd = *(Update*)radio.DATA; //assume radio.DATA actually contains our struct and not something else
//
//        if (DEBUG) { 
//          Serial.print('Key: '); Serial.println(upd.key); 
//          Serial.print('Val: '); Serial.println(upd.value);
//        }
//        
//        if (radio.ACKRequested()) { radio.sendACK(); }
//      
//        if (upd.key == "normal") {
//            mode = MODE_NORMAL;
//            
//        } else if (upd.key == "config") {
//            mode = MODE_CONFIG;
//             
//        } else if (upd.key == "interval") {
//            TRANSMITPERIOD = upd.value;
//            
//        } else if (upd.key == "show") {
//            //ledBlink(255,255,255, 100, 10);
//        }
//      }
    }
  }  
}



//===================================================
// Sending
//===================================================

// Define payload
typedef struct {
  uint8_t volume; // Volume
  uint8_t battery; // Battery voltage 
  uint8_t rssi;    // rssi
} Payload;
Payload payload;


// sendConfig
void sendConfig() {
  // Battery Level
  payload.battery = 5; //(int) getBatteryLevel();
  // Volume 
  payload.volume  = 3; // (int) getSoundPressure(); 
  // RSSI from node to server
  payload.rssi  = 2; // gatewayRSSI;
  // Send
  sendPayload();
}

// sendReading
void sendReading() {
  // Battery Level
  payload.battery = (int) getBatteryLevel();
  // Volume 
  payload.volume  = (int) getSoundPressure(); 
  // RSSI from node to server
  payload.rssi  = 0;
  // Send
  sendPayload();
}

void sendPayload() {  
  // Print payload
  if (DEBUG) {
    Serial.print("Sending payload ("); Serial.print(sizeof(payload)); Serial.print(" bytes) to node: "); Serial.println(dest);
    Serial.print(" - Battery="); Serial.print(payload.battery);
    Serial.print(" - Volume="); Serial.print(payload.volume);
    Serial.print(" - Gateway RSSI="); Serial.print(payload.rssi);
    Serial.println(")");
  }
  // Send payload
  if (radio.sendWithRetry(dest, (const uint8_t*) &payload, sizeof(payload))) {
  //if (radio.sendWithRetry(dest, "TEST", sizeof("TEST"))) {
    modeTimer = millis();
    myDelay(1000);
    if (DEBUG) Serial.println(" Acknoledgment received!");
    ledBlink(0,255,0, 100, 2);
  } else {
    if (DEBUG) Serial.println(" No Acknoledgment after retries");
    myDelay(1000);
    ledBlink(255,0,0, 100, 2);
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
  Serial.print("I am node: "); Serial.println(NODEID);
  Serial.print("on network: "); Serial.println(NETWORKID);
  Serial.print("talking to gateway "); Serial.print(GATEWAYID);
  Serial.print(" every "); Serial.print(TRANSMITPERIOD); Serial.println("ms");
  
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
// Sensor Sound Level
//===================================================

bool initMicrophone() {
  if (!I2S.begin(I2S_PHILIPS_MODE, 16000, 32)) {
      if (DEBUG) Serial.println("Failed to initialize I2S!");
      return false;
  }
  return true;
}

float getSoundPressure() {
  
  if (DEBUG) Serial.println("Getting audio level");
  
  int tries = 40000;
  float meanval = 0;
  
  for (int i=0; i<SAMPLES; i++) {

    int sample = 0; 
    while (((sample == 0) || (sample == -1)) && tries > 0 ) { // only try 50 times after that stop
       
      sample = I2S.read();
      tries--;
    }

    if (tries <= 0) // restart MC if no samples
      NVIC_SystemReset();
    // convert to 18 bit signed
    sample >>= 14; 
    samples[i] = sample;
    meanval += samples[i];// ok we have the samples, get the mean (avg)
  }
  meanval /= SAMPLES;
  
  // subtract it from all samples to get a 'normalized' output
  int maxsample = 0;
  for (int i=0; i<SAMPLES; i++) {
    samples[i] -= meanval;
    if (samples[i] > maxsample) maxsample = samples[i];
  }

  int newdB = 20 * log10((float)maxsample / (float)ADC_SOUND_REF) + DB_SOUND_REF;
  return newdB;
//  return 10.0;
}

//===================================================
// Sensor Battery Level
//===================================================

float getBatteryLevel() {
  if (DEBUG) Serial.println("Getting battery voltage");
  
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  return measuredvbat*10;
}


//===================================================
// Sensor Battery Level
//===================================================

void setColor(int red, int green, int blue) {
  if (USELED) {
    #ifdef COMMON_ANODE
      red = 255 - red;
      green = 255 - green;
      blue = 255 - blue;
    #endif
    analogWrite(redPin, red);
    analogWrite(greenPin, green);
    analogWrite(bluePin, blue);  
  }
}

//===================================================
// Blink
//===================================================

void ledBlink(int red, int green, int blue, int delayms, int numberTimes) {
  for (int x=0; x < numberTimes; x++) {
    setColor(red, green, blue);
    myDelay(delayms);
    if (mode == MODE_NORMAL) {
      setColor(0,0,0);
    } else {
      setColor(0,0,120);
    }
    myDelay(delayms);
  }
}

//===================================================
// Die
//===================================================

void die() {
  while (true) {
    setColor(255,0,0); 
    delay(1000);
  } 
}

void myDelay(int waitFor) {
  delay(waitFor);
//   int start = millis();
//    while (true) { 
//      int remianing = (waitFor - (millis() - start));
//      Serial.println(remianing);
//      if (remianing < 1) return;
//    }
}

