//Libraries
#include "LSM6DS3.h"
#include "Wire.h"
#include "mic.h"

// Settings
#define DEBUG 1
#define SAMPLES 800                   

//Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);

// Microphone Configuration
mic_config_t mic_config{
  .channel_cnt = 1,
  .sampling_rate = 16000,
  .buf_size = 1600,
  //.debug_pin = LED_BUILTIN
};

//Create an instance of class NRF52840
NRF52840_ADC_Class Mic(&mic_config);

int16_t recording_buf[SAMPLES];
volatile uint8_t recording = 0;
volatile static bool record_ready = false;

void setup() {

  Serial.begin(9600);
  while (!Serial) {delay(10);}

  Mic.set_callback(audio_rec_callback);

  if (!Mic.begin()) {
    Serial.println("Mic initialization failed");
    while (1); }
    
 if (myIMU.begin() != 0) {
        Serial.println("Device error"); } 
    
}

void loop() {
 
  if (record_ready)
  {

  for (int i = 0; i < SAMPLES; i++) {

  //int16_t sample = filter.step(recording_buf[i]);
  int16_t sample = recording_buf[i];

  //Microphone (Blue)    // For Edge Impulse to work, the output must be in the form "x,y", hence all unnecessary text was removed...
  Serial.print(sample);  // The difference between Serial.print and Serial.println is that println moves the next print to a new line...
  Serial.print(",");     // So you would use print until you reach your last variable, then use println so that the next reading appears on a fresh line...
  
  //Thermometer (Green)
  Serial.println(myIMU.readTempF(), 4); // Hence why I use println here... (by the way, the 4 indicates the number of decimal places of the reading)

    delay(160);
  }

  record_ready = false;
  }
}

static void audio_rec_callback(uint16_t *buf, uint32_t buf_len) {

  static uint32_t idx = 0;
  // Copy samples from DMA buffer to inference buffer
  {
    for (uint32_t i = 0; i < buf_len; i++) {

      // Convert 12-bit unsigned ADC value to 16-bit PCM (signed) audio value
      recording_buf[idx++] = buf[i];
      if (idx >= SAMPLES){
      idx = 0;
      recording = 0;
      record_ready = true;
      break;
     }
    }
  }
}
