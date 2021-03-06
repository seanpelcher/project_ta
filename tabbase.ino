// Libraries (Necessary to use certain commands, do not delete these...)
#include "LSM6DS3.h"
#include "Wire.h"
#include "mic.h"

// Settings
#define DEBUG 1                       // Enable pin pulse during ISR
#define SAMPLES 800                   

// Create an instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);        // I2C device address 0x6A

// Microphone Configuration
mic_config_t mic_config{
  .channel_cnt = 1,
  .sampling_rate = 16000,
  .buf_size = 1600,
  //.debug_pin = LED_BUILTIN          // This line activates the LED light, so leave as a comment to save energy!
};

// Create an instance of class NRF52840
NRF52840_ADC_Class Mic(&mic_config);

int16_t recording_buf[SAMPLES];
volatile uint8_t recording = 0;
volatile static bool record_ready = false;

void setup() {

  Serial.begin(9600);
  while (!Serial) {delay(10);}        // The number inside this delay command determines the time before setup begins

  Mic.set_callback(audio_rec_callback);

  if (!Mic.begin()) {
    Serial.println("Mic initialization failed");
    while (1); }
    
 if (myIMU.begin() != 0) {
        Serial.println("Device error"); } 
    
 else { Serial.println("Device OK!");                   // The entire else statement could be deleted to avoid having to print extra lines
        Serial.println("Mic initialization done."); }
}

void loop() {

  if (record_ready)
  {
  Serial.println("Finished sampling");

  for (int i = 0; i < SAMPLES; i++) {

  //int16_t sample = filter.step(recording_buf[i]);
  int16_t sample = recording_buf[i];

  //Microphone                            // To not include a certain sensor in the serial monitor output, simply comment all associated lines
  Serial.print("\nMicrophone:\n");
  Serial.print(" Noise Value = ");
  Serial.println(sample);
  
   //Accelerometer
    Serial.print("\nAccelerometer:\n");
    Serial.print(" X1 = ");
    Serial.println(myIMU.readFloatAccelX(), 4);
    Serial.print(" Y1 = ");
    Serial.println(myIMU.readFloatAccelY(), 4);
    Serial.print(" Z1 = ");
    Serial.println(myIMU.readFloatAccelZ(), 4);

    //Gyroscope
    Serial.print("\nGyroscope:\n");
    Serial.print(" X1 = ");
    Serial.println(myIMU.readFloatGyroX(), 4);
    Serial.print(" Y1 = ");
    Serial.println(myIMU.readFloatGyroY(), 4);
    Serial.print(" Z1 = ");
    Serial.println(myIMU.readFloatGyroZ(), 4);

    //Thermometer
    Serial.print("\nThermometer:\n");
    Serial.print(" Degrees C1 = ");
    Serial.println(myIMU.readTempC(), 4);
    Serial.print(" Degrees F1 = ");
    Serial.println(myIMU.readTempF(), 4);

    delay(250);                             // The number inside this delay command determines the time between each reading
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
