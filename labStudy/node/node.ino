// **********************************************************************************
//
// This script is designed to test the RSSI - Requires a not to be sending messages
// periodically. The output can be plotted using the serial monitor.
//                       
// **********************************************************************************

#include <RFM69.h>              // https://www.github.com/lowpowerlab/rfm69
#include <I2S.h>

// **********************************************************************************


#define RXID          10 // This device i.e. the receiver of the message from TX
#define TXID          11 // The ID of the device sending messages
#define RELAYID       21 // The ID of the RFM69 to BLE Relay node
#define NETWORKID     50  

#define RADIO_POWER   31   // power output ranges from 0 (5dBm) to 31 (20dBm)

#define VBATPIN A7
// Are you using the RFM69 Wing? Uncomment if you are.
//#define USING_RFM69_WING 

// The transmision frequency of the baord. Change as needed.
#define FREQUENCY      RF69_433MHZ //RF69_868MHZ // RF69_915MHZ

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

//===================================================
// Setup
//===================================================

void setup() {
  
  //  Wait for Serial
  if (LOG) Serial.begin(SERIAL_BAUD);
  if (LOG) { while (!Serial) { delay(100); } }
     
  // Initialize the radio
  radio.initialize(FREQUENCY,RXID,NETWORKID);
  #ifdef IS_RFM69HW_HCW
    radio.setHighPower(); //must include this only for RFM69HW/HCW!
  #endif
 
  // Set radio Power level  
  radio.setPowerLevel(RADIO_POWER); 

  // Init Microphone
  if (!I2S.begin(I2S_PHILIPS_MODE, 16000, 32)) {
    Serial.println("Failed to initialize I2S!");
    while (1); // do nothing
  }
}

#define ADC_SOUND_REF 65
#define DB_SOUND_REF 41

//===================================================
// Main loop
//===================================================

#define num_samples 1024

int sample_index = 0;
int samples[num_samples];
int failure_count = 0;

int rssi = 0;
int last_rssi = millis();

void loop() {
    
    // Listen for messages from gateway
    if (radio.receiveDone()) {
      if (radio.SENDERID == TXID) {
          if (LOG) Serial.println(radio.RSSI);
          rssi = radio.RSSI;
          last_rssi = millis();
      }
    }

    if (failure_count > 10000) {
      if (DEBUG) Serial.print("Resetting Microphone...");
      NVIC_SystemReset();
      delay(500);
      failure_count = 0;
    }
    
    if (sample_index < num_samples) {
      
      int sample = I2S.read();
      if (!((sample == 0) || (sample == -1))) {
        // convert to 18 bit signed
        sample >>= 14; 
        samples[sample_index] = sample;
        sample_index++;
      } else {
        failure_count++;
      }

    } else {
      
      float meanval = 0;
      for (int i=0; i<num_samples; i++) { meanval += samples[i];  }
      meanval /= num_samples;
           
      // subtract it from all sapmles to get a 'normalized' output
      for (int i=0; i<num_samples; i++) {
        samples[i] -= meanval;
      }
     
      // find the 'peak to peak' max
      float maxsample, minsample;
      minsample = 100000;
      maxsample = -100000;
      for (int i=0; i<num_samples; i++) {
        minsample = min(minsample, samples[i]);
        maxsample = max(maxsample, samples[i]);
      }
//      float peakTopeak = maxsample - minsample;
//      if (DEBUG) Serial.print("Peak2Peak: "); 
//      if (LOG) Serial.println(peakTopeak); 
      
      int dB = 20 * log10((float)maxsample / (float)ADC_SOUND_REF) + DB_SOUND_REF;

      // Reset sampling
      sample_index = 0;
      failure_count = 0;

      if (millis() - last_rssi > 2000) {
        rssi = 100;
      }
      
      // Send data to relay node
      sendPayload(rssi, dB);
  
      if (DEBUG) Serial.print("dB: "); 
      if (LOG) Serial.println(dB); 
    }

    
  
}


// Define payload
typedef struct {
  uint8_t volume; // Volume
  uint8_t battery; // Battery voltage 
  uint8_t rssi;    // rssi
  uint8_t actual_rssi;    // rssi
} Payload;
Payload payload;

// Send Payload
void sendPayload(int sig, float db) {

  float sendDB   = min(100, ((db-35) / 100) * 100);
  float sendRSSI = 100 - min(100, (abs((sig+25)) / 90.0) * 100);
//  Serial.print("sendDB "); Serial.println((int) sendDB);
//  Serial.print("sendRSSI "); Serial.println((int) sendRSSI);

  payload.battery = getBatteryLevel();
  payload.volume  = (int) round(sendDB); 
  payload.rssi    = (int) round(sendRSSI);
  payload.actual_rssi = abs(rssi);
  //radio.send(RELAYID, "SEND Test", sizeof("SEND Test"));
  radio.send(RELAYID, (const uint8_t*) &payload, sizeof(payload));
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



float getBatteryLevel() {
  if (DEBUG) Serial.println("Getting battery voltage");
  
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  return measuredvbat*10;
}