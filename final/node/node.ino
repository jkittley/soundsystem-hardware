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
#define NETWORKID     200  

#define RADIO_POWER   31   // power output ranges from 0 (5dBm) to 31 (20dBm)

// Are you using the RFM69 Wing? Uncomment if you are.
//#define USING_RFM69_WING 

// The transmision frequency of the baord. 
#define FREQUENCY      RF69_433MHZ 

// Uncomment if this board is the RFM69HW/HCW not the RFM69W/CW
#define IS_RFM69HW_HCW

// Serial board rate - just used to print debug messages
#define SERIAL_BAUD   115200

bool LOG = false;    // Log data out
bool DEBUG = false;  // Show debug messages

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

#define ADC_SOUND_REF 65
#define DB_SOUND_REF 41

int configButton = A3;


#define num_samples 256

int sample_index = 0;
int samples[num_samples];
int failure_count = 0;


// Define payload
typedef struct {
  uint8_t volume; // Volume
  uint8_t battery; // Battery voltage 
} TXPayload;
TXPayload sendPayload;

// Define payload
typedef struct {
  uint8_t volume; // Volume
  uint8_t rssi;    // rssi
} RXPayload;
RXPayload receivePayload;


unsigned long config_timer = 0;
int is_in_config = false;
unsigned long stayInConfig = 1000 * 60 * 1;

int redPin = 5;
int greenPin = 10;
int bluePin = 11;


//===================================================
// Setup
//===================================================

void setup() {
  
  //  Wait for Serial
  if (LOG || DEBUG) Serial.begin(SERIAL_BAUD);
  if (LOG || DEBUG) { while (!Serial) { delay(100); } }
     
  // Initialize the radio
  radio.initialize(FREQUENCY, NODEID, NETWORKID);
  #ifdef IS_RFM69HW_HCW
    radio.setHighPower(); //must include this only for RFM69HW/HCW!
  #endif
 
  // Set radio Power level  
  radio.setPowerLevel(RADIO_POWER); 

  // Init Microphone
  if (!I2S.begin(I2S_PHILIPS_MODE, 16000, 32)) {
    if (DEBUG) Serial.println("Failed to initialize I2S!");
    while (1); // do nothing
  }

  pinMode(configButton, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(configButton), onButtonChange, HIGH);
  
}



//===================================================
// Main loop
//===================================================

void onButtonChange() {
  is_in_config = true;
  config_timer = stayInConfig;
  setColor(0,0,255);

  if (radio.sendWithRetry(RELAYID,"CHOOSEME", sizeof("CHOOSEME"))) {
      if (DEBUG) Serial.println("CHOOSEME ACK received");
  }
               
}


void loop() {

    

    
    // Listen for messages 
    if (radio.receiveDone()) {
      // Messages from the base station
      if (radio.SENDERID == BASEID) {
          if (LOG) Serial.println("Message from basestation");
            if (radio.DATALEN == sizeof(RXPayload)) {
               receivePayload = *(RXPayload*)radio.DATA; 
               if (DEBUG) Serial.println(receivePayload.volume);
               if (DEBUG) Serial.println(receivePayload.rssi);

               if (radio.sendWithRetry(RELAYID, (const uint8_t*) &receivePayload, sizeof(receivePayload))) {
                  //success   
                  config_timer = millis() + stayInConfig;
               }
            }
      } 
    }

    if (stayInConfig < millis()) {
      is_in_config = false;
      setColor(0,0,0);
    }


    if (failure_count > 10000) {
      if (DEBUG) Serial.print("Resetting Microphone...");
        NVIC_SystemReset();
//      delay(500);
//      I2S.begin(I2S_PHILIPS_MODE, 16000, 32);
//      delay(500);
        failure_count = 0;
    }
    
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

    } else {
      
      if (DEBUG) Serial.println("Sending");
      
      float meanval = 0;
      float minsample = 100000;
      float maxsample = -100000;
      for (int i=0; i<num_samples; i++) { meanval += samples[i];  }
      meanval /= num_samples;
           
      // subtract it from all sapmles to get a 'normalized' output
      for (int i=0; i<num_samples; i++) {
        samples[i] -= meanval;
      }
     
      // find the 'peak to peak' max
       maxsample, minsample;
      
      for (int i=0; i<num_samples; i++) {
        minsample = min(minsample, samples[i]);
        maxsample = max(maxsample, samples[i]);
      }
      
      int dB = 20 * log10((float)maxsample / (float)ADC_SOUND_REF) + DB_SOUND_REF;

      // Reset sampling
      sample_index = 0;
      failure_count = 0;

    
      // Send data to relay node
      sendVolume(dB);

    }
}




// Send Payload
void sendVolume(float db) {

  float min_db = 35; // Background level
  float max_db = 90; // Everything above this is 100%
   
  float sendDB   = max(0, min(100, 100 * ( (db-min_db) / (max_db-min_db) ) ));

  if (DEBUG) Serial.print(" <--> Vol dB: "); 
  if (DEBUG) Serial.print(db); 
  if (DEBUG) Serial.print(" | %: "); 
  if (DEBUG) Serial.println(sendDB); 

  sendPayload.battery = 0;
  sendPayload.volume  = (int) round(sendDB); 
  
  //radio.send(RELAYID, "SEND Test", sizeof("SEND Test"));
  radio.send(RELAYID, (const uint8_t*) &sendPayload, sizeof(sendPayload));
}

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

