///// Integrated Code Thermistor and Sound Power Level /////

// Libraries
#include "LSM6DS3.h"         // adds the library "LSM6DS3" to the code
#include "Wire.h"            // adds the library "Wire" to the code
#include "mic.h"             // adds the library "mic" to the code
#include "ArduinoBLE.h"      // adds the library "ArduinoBLE" to the code

// Thermistor Code
float slopeArray[10];        // array of size 10 to store past 10 slope values

// BPM Code for Thermistor
int x = 0;                   // sets integer "x" equal to 0
int LastTime = 0;            // sets integer "LastTime" equal to 0
bool BPMTiming = false;      // sets bool "BPMTiming" equal to false
bool BeatComplete = false;   // sets bool "BeatComplete" equal to false
int BPM = 0;                 // sets integer "BPM" equal to 0
#define UpperThreshold 0.5   // 24.5 (alternative value)           // threshold above which the signal is considered a breath for thermistor
#define LowerThreshold -0.2  // 24 (alternative value, for normal) // threshold below which a breath is considered complete for thermistor
int Signal;                  // holds the incoming raw data, signal value can range from 0 to 1024

// Time for Thermistor
unsigned long myTime;        // creates the long integer "myTime"
unsigned long startMillis;   // creates the long integer "startMillis"

// Thermistor Definitions
#define THERMISTORPIN A3            // which analog pin to connect   // MAKE SURE TO CHANGE THE PIN ACCORDING TO THE THERMISTOR PLACEMENT ON THE BOARD         
#define THERMISTORNOMINAL 10000     // resistance at 25 Celsius
#define TEMPERATURENOMINAL 22.778   // temperature for nominal resistance (almost always 25 Celsius)
#define NUMSAMPLES 3                // how many samples to take and average, more takes longer but is more smooth
#define BCOEFFICIENT 3950           // The beta coefficient of the thermistor (usually 3000-4000)
#define SERIESRESISTOR 10000        // the value of the 'other' resistor   

// Low-Pass Filter Variables for Thermistor
#define LPF_ALPHA 0.1                // filter constant (0 < alpha < 1)
float prevValue = 0;                 // sets float "prevValue" equal to 0
float prevTime = 0;                  // sets float "prevTime" equal to 0
float prevSlope = 0;                 // sets float "prevSlope" equal to 0

int samples[NUMSAMPLES];             // creates the integer "samples" based on the inputted NUMSAMPLES           

// Bluetooth (not necessary to breakdown, just include)
BLEService echoService("00000000-0000-1000-8000-00805f9b34fb");
BLEStringCharacteristic charac ("741c12b9-e13c-4992-8a5e-fce46dec0bff", BLERead | BLEWrite | BLENotify, 40);
BLEDescriptor Descriptor("beca6057-955c-4f8a-e1e3-56a1633f04b1","Descriptor");

// Variables
int UpperLimit;
String UpperLimitW;
String UpperLimitInput;
int LowerLimit;
String LowerLimitW;
String LowerLimitInput;
int Age;
String AgeW;
String AgeInput;
int Mode;
String ModeW;
String ModeInput;
int counter;
int abnormalcounter = 652;// 15 seconds -- 1000; 1 second ~ 67; 10 seconds ~ 670
int abnormalcounter1 = 652;
int abnormalcounter2 = 652;
int abnormalcounter3 = 652;
int abnormalcounter4 = 652;
int caution;
int escape;
int gate;
int door;
int uppertunnel;
int lowertunnel;
int middletunnel;
int detour;

int numberbpm;
String wordbpm;
int numberbpmmic;
String wordbpmmic;
int numbertemp;
String wordtemp;
String phonedisplay;
int numberaverage;
String wordaverage;

// Microphone Code
// Settings
#define DEBUG 1
#define SAMPLESmic 800                   

// Low-Pass Filter Variables for Mic
#define LPF_ALPHAmic 0.1      // filter constant (0 < alpha < 1)
float prevValuemic = 0;       // sets float "prevValuemic" equal to 0

// BPM Code for Mic
int xmic = 0;                   // sets integer "xmic" equal to 0
int LastTimemic = 0;            // sets integer "LastTimemic" equal to 0
bool BPMTimingmic = false;      // sets bool "BPMTimingmic" equal to false
bool BeatCompletemic = false;   // sets bool "BeatCompletemic" equal to false
int BPMmic = 0;                 // sets integer "BPMmic" equal to 0
#define UpperThresholdmic 0.1   // 0.04 (alternative value)             // threshold above which the signal is considered a breath
#define LowerThresholdmic 0.01  // 0.01 (alternative value, for normal) // threshold below which a breath is considered complete 
int Signalmic;                  // holds the incoming raw data, signal value can range from 0 to 1024

// Time for Mic
unsigned long myTimemic;        // creates the long integer "myTimemic"
unsigned long startMillismic;   // creates the long integer "startMillismic"


// Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);

// Microphone Configuration
mic_config_t mic_config{
  .channel_cnt = 1,
  .sampling_rate = 16000,
  .buf_size = 1600,
  //.debug_pin = LED_BUILTIN
};

// Create an instance of class NRF52840
NRF52840_ADC_Class Mic(&mic_config);

int16_t recording_buf[SAMPLESmic];
volatile uint8_t recording = 0;
volatile static bool record_ready = false;

// Setting Up TrachAlert
void setup() {                                   // setup code begins

  delay(5000);                                   // wait 5 seconds         
  Serial.println("Initializing TrachAlert...");  // display a loading message to the user
  delay(5000);                                   // wait 5 seconds

  Mic.set_callback(audio_rec_callback);          // start to initialize the microphone

  detour = 3;                                    // set "detour" to open (3)
  gate = 3; // 3 = open, 1 = closed              // set "gate" to open (3)
  door = 3; // 3 = open, 1 = closed              // set "door" to open (3)
  uppertunnel = 3;                               // set "uppertunnel" to open (3)
  lowertunnel = 3;                               // set "lowertunnel" to open (3)
  middletunnel = 3;                              // set "middletunnel" to open (3)
  
  Serial.begin(9600);  // tells the arduino to get ready to exchange messages with the serial monitor at a data rate of 9600 bits per second

  if (!Mic.begin()) {                               // if there is a problem with the microphone...
    Serial.println("Mic initialization failed.");   // tell the user that...
    while (1); }                                    // and do not continue running the code!

  if(!BLE.begin()){                                 // if there is a problem with the bluetooth...
    Serial.println("BLE failed.");                  // tell the user that...
    while(1); }                                     // and do not continue running the code!
  
 if (myIMU.begin() != 0) {                          // if there is a problem with the accelerometer system...
       Serial.println("Device error."); }           // tell the user that, but code can continue!
    
 else { Serial.println("Device OK!");                   // otherwise, tell the user that everything is ok...
        Serial.println("Mic initialization done."); }   // and that the microphone initilization is done!

  // Bluetooth Initilization (not necessary to breakdown, just include)
  BLE.setLocalName("TrachAlert");         // name the TrachAlert device
  BLE.setAdvertisedService(echoService);
  charac.addDescriptor(Descriptor);
  echoService.addCharacteristic(charac);
  BLE.addService(echoService);
  BLE.advertise();                        // begin bluetooth communication
  
  startMillis = millis();                 // set the variable "startMillis" equal to the current time

  for (int t = 0; t < 10; t++) {
    slopeArray[t] = 0;
  }

  Serial.println("TrachAlert initialized.");                              // tell the user that the TrachAlert has initialized...
  delay(2000);                                                            // wait 2 seconds...
  Serial.println("Attempting to establish a Bluetooth connection..."); }  // then prepare for a Bluetooth connection.

// Running the TrachAlert
void loop() {                                                     // the main loop starts and cannot be exited

if (record_ready) {                        // start of record_ready loop            
  for (int s = 0; s < SAMPLESmic; s++) {   // start of sampling loop, SAMPLESmic is set to 800 by default, loop continues as long as "s" remains less than SAMPLESmic

  uint8_t i;     // creates the unsigned integer of 8 bits "i"
  float average; // creates the float "average"

  // Take N samples in a row, with a slight delay
  for (i = 0; i < NUMSAMPLES; i++) {       // NUMSAMPLES is set to 3 by default, loop continues as long as "i" remains less than NUMSAMPLES
   samples[i] = analogRead(THERMISTORPIN); // each thermistor reading constitutes a sample
  }
    
  // Average all the samples out
  average = 0;                          // sets float "average" equal to 0
  for (i = 0; i < NUMSAMPLES; i++) {    // NUMSAMPLES is set to 3 by default, loop continues as long as "i" remains less than NUMSAMPLES
    average += samples[i];              // adds the value of the sample to "average"
    delay(2);                           // a very slight delay to allow for calculation
  }
  average /= NUMSAMPLES;                // divide "average" by NUMSAMPLES to get the average raw thermistor reading

  // Convert the value to resistance
  average = 1023 / average - 1;         // divide 1023 by "average" subtracted by 1
  average = SERIESRESISTOR / average;   // divide "SERIESRESISTOR" (which is set to 10000) by the previous answer
  
  // Filter the value with a low-pass filter
  float filteredValue = (LPF_ALPHA * average) + ((1 - LPF_ALPHA) * prevValue);
  prevValue = filteredValue;

  // Calculate the Steinhart value from the filtered value
  float steinhart;                                   // creates the float "steinhart"
  steinhart = filteredValue / THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                        // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                         // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15);  // + (1/To)
  steinhart = 1.0 / steinhart ;                      // invert
  steinhart -= 273.15;                               // convert absolute temp to C
  
  // Output the Steinhart value and slope
  myTime = millis();                                                     // set myTime equal to the current time in milliseconds
  float slope = (steinhart - prevSlope) / ((myTime - prevTime) / 1000);  // determine the slope of the thermistor readings
  // float slope = (steinhart - prevSlope) / (myTime - prevTime);
    prevTime = myTime;                                                // set prevTime equal to myTime...
    prevSlope = steinhart;                                            // and set prevSlope to the last steinhart reading to prepare for the next slope calculation

// 
int value1 = slope;                         // sets the integer "value1" equal to "slope"
  if (value1 > UpperThreshold) {            // if "value1" (or the slope) is greater than the upper threshold of 0.5...
    
    if (BeatComplete) {                     // and if BeatComplete is set to true...
      BPM = millis() - LastTime;            // set "BPM" equal to the current time in milliseconds subtracted by the LastTime...
      BPM = int(60 / (float(BPM) / 1000));  // then divide "BPM" by 1000 and divide 60 by that answer
      BPMTiming = false;                    // set BPMTiming to false...
      BeatComplete = false;                 // and set BeatComplete to false...
    }
    
    if (BPMTiming == false) {               // if BPMTiming is false (which it should have just been set to...)
      LastTime = millis();                  // set LastTime equal to the current time in milliseconds...
      BPMTiming = true;                     // and set BPMTiming to true
    }
  }
  
  if ((value1 < LowerThreshold) & (BPMTiming)) // if "value1" (or the slope) is less than the lower threshold of -0.2
    BeatComplete = true;                       // set BeatComplete equal to true


    delay(6);  // a very slight delay to allow for calculation

  //int16_t sample = filter.step(recording_buf[i]);
  int16_t samplemic = recording_buf[s];

unsigned long myTime;  // creates the long integer myTime
myTime = millis();     // set myTime equal to the current time in milliseconds

  int16_t dbFS = 20*log10((abs(samplemic)+ 0.01)/32767); 
  // Serial.println(dbFS); // second command may be useful for displaying dBFS instead of PCM
  // added +0.01 to the abs value to ensure that it is never zero, even for small value. 
  // Serial.print("; ");


// Power Value in Linear Scale 
  float power = pow(10, (dbFS - (-26)) / 10.0); // sensitivity -26 dBFS --> REFERENCE LEVEL
  
// Filter the value with a low-pass filter
  float filteredValuemic = (LPF_ALPHAmic * power) + ((1 - LPF_ALPHAmic) * prevValuemic);
  prevValuemic = filteredValuemic; // sets prevValuemic equal to filteredValuemic


int value = filteredValuemic;                       // sets the integer "value" equal to filteredValuemic
  if (value > UpperThresholdmic) {                  // if "value" is greater than the upper threshold of 0.1...
    
    if (BeatCompletemic) {                          // and if BeatCompletemic is set to true...
      BPMmic = millis() - LastTimemic;              // set "BPMmic" equal to the current time in milliseconds subtracted by the LastTimemic...
      BPMmic = int(60 / (float(BPMmic) / 1000));    // then divide "BPMmic" by 1000 and divide 60 by that answer
      BPMTimingmic = false;                         // set BPMTimingmic to false...
      BeatCompletemic = false;                      // and set BeatCompletemic to false...
    }
    
    if (BPMTimingmic == false) {                    // if BPMTimingmic is false (which it should have been set to...)
      LastTimemic = millis();                       // set LastTimemic equal to the current time in milliseconds...
      BPMTimingmic = true;                          // and set BPMTimingmic to true
    }
  }
    
  if ((value < LowerThresholdmic) & (BPMTimingmic)) // if "value" is less than the lower threshold of 0.01
    BeatCompletemic = true;                         // set BeatCompletemic to true

Serial.print(myTime);                               // TrachAlert will then print the current time...
Serial.print(",");
Serial.print(BPMmic);                               // the BPM as determined by the microphone...
Serial.print(",");
Serial.println(BPM);                                  // and the BPM as determined by the thermistor...

  }  // end of sampling loop
  
  }  // end of record_ready loop
  
record_ready = false;
  
} // end of main loop





// Necessary to include for the code to work but not necessary to understand in full
static void audio_rec_callback(uint16_t *buf, uint32_t buf_len) {

  static uint32_t idx = 0;
  // Copy samples from DMA buffer to inference buffer
  {
    for (uint32_t s = 0; s < buf_len; s++) {

      // Convert 12-bit unsigned ADC value to 16-bit PCM (signed) audio value
      recording_buf[idx++] = buf[s];
      if (idx >= SAMPLESmic){
      idx = 0;
      recording = 0;
      record_ready = true;
      break;
     }
    }
  }
}
