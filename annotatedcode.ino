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
#define LPF_ALPHAmic 0.1      // Filter constant (0 < alpha < 1)
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

void setup() {

  delay(5000);
  Serial.println("Initializing TrachAlert...");
  delay(5000);

  Mic.set_callback(audio_rec_callback);

  detour = 3;
  gate = 3; // 3 = open, 1 = closed
  door = 3; // 3 = open, 1 = closed
  uppertunnel = 3;
  lowertunnel = 3;
  middletunnel = 3;
  
  Serial.begin(9600);

  if (!Mic.begin()) {
    Serial.println("Mic initialization failed.");
    while (1); }

  if(!BLE.begin()){
    Serial.println("BLE failed.");
    while(1);
  }
  
 if (myIMU.begin() != 0) {
       Serial.println("Device error."); } 
    
 else { Serial.println("Device OK!");
        Serial.println("Mic initialization done.");
       // Serial.println("Microphone,Accelerometer,Temperature"); }
}

  BLE.setLocalName("TrachAlert");
  BLE.setAdvertisedService(echoService);
  charac.addDescriptor(Descriptor);
  echoService.addCharacteristic(charac);
  BLE.addService(echoService);
  BLE.advertise();
  
  startMillis = millis();

  for (int t = 0; t < 10; t++) {
    slopeArray[t] = 0;
  }

  Serial.println("TrachAlert initialized.");
  delay(2000);
  Serial.println("Attempting to establish a Bluetooth connection...");
}

void loop() {
BLEDevice central = BLE.central();

if(central) {
  Serial.println("Bluetooth connection established.");
  Serial.print("Connected to central: ");
  Serial.println(central.address());

while(central.connected()){
if(gate > 2){
  Serial.println("////////// TrachAlert System //////////");
  String title = "TrachAlert System";
  charac.writeValue(title);
  Serial.println("Manual Mode (1) or Age Mode (2)?");
  String manual = "Manual Mode (1) or Age Mode (2)";
  charac.writeValue(manual);
  while (central.connected() && middletunnel > 2) {
    if(charac.written()){
      ModeInput = charac.value();
      Mode = ModeInput.toInt();
      middletunnel = 1;
    }
  }

  if (Mode == 1) {
    Serial.println("What should the upper limit for the respiratory rate be?");
  String firstquestion = "Input Upper Limit: ";
  charac.writeValue(firstquestion);
  while (central.connected() && uppertunnel > 2) {
    if(charac.written()){
      UpperLimitInput = charac.value();
      UpperLimit = UpperLimitInput.toInt();
      uppertunnel = 1;
    }
  }
  Serial.print("Upper respiratory rate limit set to: ");
  String firstresponse = "Upper Limit set to: ";
  charac.writeValue(firstresponse);
  Serial.print(UpperLimit);
  UpperLimitW = String(UpperLimit);
  charac.writeValue(UpperLimitW);
  Serial.println(" BPM");
  Serial.println("And what should the lower limit for the respiratory rate be?");
  String secondquestion = "Input Lower Limit: ";
  charac.writeValue(secondquestion);
  while (central.connected() && lowertunnel > 2) {
    if(charac.written()){
      LowerLimitInput = charac.value();
      LowerLimit = LowerLimitInput.toInt();
      lowertunnel = 1;
    }
  }
  Serial.print("Lower respiratory rate limit set to: ");
  String secondresponse = "Lower Limit set to: ";
  charac.writeValue(secondresponse);
  Serial.print(LowerLimit);
  LowerLimitW = String(LowerLimit);
  charac.writeValue(LowerLimitW);
  Serial.println(" BPM");
  Serial.println("Thank you for choosing TrachAlert. Respiratory rate measurement will start in 5 seconds.");
  String thanks = "Thank you.";
  charac.writeValue(thanks);
  counter = 1000; // Initial counter value
  caution = 300; //Initial caution value
  escape = 0; //Initial escape value;
  delay(5000); // Seconds before respiratory rate measurement begins
  gate = 1;
  Mode = 0;
  }

  if (Mode == 2) {
  Serial.println("What is the age of the patient? (if patient is less than 1 year old, input 0)");
  String firstquestion = "Input Age in years:";
  charac.writeValue(firstquestion);
  while (central.connected() && uppertunnel > 2) {
    if(charac.written()){
      AgeInput = charac.value();
      Age = AgeInput.toInt();
      uppertunnel = 1;
    }
  }
  Serial.print("Age set to ");
  String firstresponse = "Age set to: ";
  charac.writeValue(firstresponse);
  Serial.print(Age);
  AgeW = String(Age);
  charac.writeValue(AgeW);
  Serial.println(" yrs");
  
  if (Age < 1) {
    UpperLimit = 60;
    LowerLimit = 30;
  }
  if (Age >= 1 && Age < 3){
    UpperLimit = 40;
    LowerLimit = 24;
  }
  if (Age >= 3 && Age < 6){
    UpperLimit = 34;
    LowerLimit = 22;
  }
  if (Age >= 6 && Age < 12){
    UpperLimit = 30;
    LowerLimit = 18;
  }
  if (Age >= 12 && Age < 18){
    UpperLimit = 16;
    LowerLimit = 12;
  }
  if (Age >= 18 && Age < 200){
    UpperLimit = 18;
    LowerLimit = 12;
  }
  
  Serial.print("Lower respiratory rate limit set to: ");
  String secondresponse = "Lower Limit set to: ";
  charac.writeValue(secondresponse);
  Serial.print(LowerLimit);
  LowerLimitW = String(LowerLimit);
  charac.writeValue(LowerLimitW);
  Serial.println(" BPM");

delay (500); 

  Serial.print("Upper respiratory rate limit set to: ");
  String thirdresponse = "Upper Limit set to: ";
  charac.writeValue(thirdresponse);
  Serial.print(UpperLimit);
  UpperLimitW = String(UpperLimit);
  charac.writeValue(UpperLimitW);
  Serial.println(" BPM");
  
  Serial.println("Thank you for choosing TrachAlert. Respiratory rate measurement will start in 5 seconds.");
  //String thanks = "Thank you.";
  //charac.writeValue(thanks);
  counter = 1000; // Initial counter value
  caution = 300; //Initial caution value
  escape = 0; //Initial escape value;
  delay(5000); // Seconds before respiratory rate measurement begins
  gate = 1;
  Mode = 0;
}
}

        if (record_ready)
  {
    for (int s = 0; s < SAMPLESmic; s++) {
    
  uint8_t i;
  float average;

  // take N samples in a row, with a slight delay
  for (i=0; i< NUMSAMPLES; i++) {
   samples[i] = analogRead(THERMISTORPIN);
  }
  
  // average all the samples out
  average = 0;
  for (i=0; i< NUMSAMPLES; i++) {
    average += samples[i];
    delay(2);
  }
  average /= NUMSAMPLES;

  // convert the value to resistance
  average = 1023 / average - 1;
  average = SERIESRESISTOR / average;
  
  // Filter the value with a low-pass filter
  float filteredValue = (LPF_ALPHA * average) + ((1 - LPF_ALPHA) * prevValue);
  prevValue = filteredValue;

  // Calculate the Steinhart value from the filtered value
  float steinhart;
  steinhart = filteredValue / THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                        // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                         // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15);   // + (1/To)
  steinhart = 1.0 / steinhart ;                      // Invert
  steinhart -= 273.15;                               // convert absolute temp to C
  
  // Output the Steinhart value and slope
  myTime = millis();
    float slope = (steinhart - prevSlope) / ((myTime - prevTime) / 1000);
   // float slope = (steinhart - prevSlope) / (myTime - prevTime);
    prevTime = myTime;
    prevSlope = steinhart;


int value1 = slope;
  if (value1 > UpperThreshold) {
    if (BeatComplete) {
      BPM = millis() - LastTime;
      BPM = int(60 / (float(BPM) / 1000));
      BPMTiming = false;
      BeatComplete = false;
    }
    if (BPMTiming == false) {
      LastTime = millis();
      BPMTiming = true;
    }
  }
  if ((value1 < LowerThreshold) & (BPMTiming))
    BeatComplete = true;


    delay(6); // 5;

  //int16_t sample = filter.step(recording_buf[i]);
  int16_t samplemic = recording_buf[s];

unsigned long myTime;
myTime = millis(); 
  int16_t dbFS = 20*log10((abs(samplemic)+ 0.01)/32767); 
  //Serial.println(dbFS); //Second command may be useful for displaying dBFS instead of PCM
  // Added +0.01 to the abs value to ensure that it is never zero, even for small value. 
  //Serial.print("; ");


//Power Value in Linear Scale 
  float power = pow(10, (dbFS - (-26)) / 10.0); //Sensitivity -26 dBFS --> REFERENCE LEVEL
  
// Filter the value with a low-pass filter
  float filteredValuemic = (LPF_ALPHAmic * power) + ((1 - LPF_ALPHAmic) * prevValuemic);
  prevValuemic = filteredValuemic;


int value = filteredValuemic;
  if (value > UpperThresholdmic) {
    if (BeatCompletemic) {
      BPMmic = millis() - LastTimemic;
      BPMmic = int(60 / (float(BPMmic) / 1000));
      BPMTimingmic = false;
      BeatCompletemic = false;
    }
    if (BPMTimingmic == false) {
      LastTimemic = millis();
      BPMTimingmic = true;
    }
  }
  if ((value < LowerThresholdmic) & (BPMTimingmic))
    BeatCompletemic = true;
Serial.print(myTime);
Serial.print(";    ");
Serial.print(BPMmic);
Serial.print(" BPM Mic");
Serial.print(";    ");
Serial.print(BPM);
Serial.println(" BPM Temp");
numberbpm = BPM;
wordbpm = String(numberbpm);
numberbpmmic = BPMmic;
wordbpmmic = String(numberbpmmic);
String bpm_mic = String(wordbpmmic + " BPM Mic   ");
String bpm_temp = String(wordbpm + " BPM Temp");
String phonedisplay = String(bpm_mic + "; " + bpm_temp);
charac.writeValue(phonedisplay);

    //delay(1); // 5
  // check if BPMmic or BPM values exceed upper limit or drop below lower limit
if (BPM > UpperLimit || BPM < LowerLimit) {
  if (BPMmic > UpperLimit || BPMmic < LowerLimit) {
    abnormalcounter = abnormalcounter - 1;
  } else {
    abnormalcounter = 640;
  }  
}
else {
  abnormalcounter = 640; // Number of abnormal readings until alert
}

if (abnormalcounter <= 0) {
  Serial.println("Abnormal Breathing detected");
  String ABD = "Abnormal Breathing Detected!";
  charac.writeValue(ABD);
}

////////////////////////////////////NEW Added Code/////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////// 

// If Both yield zero readings for 10 consecutive seconds //NO BREATHING OR DEVICE MALFUNCTION
if ((BPM == 0 || BPM > 100) && (BPMmic == 0 || BPMmic > 100)) {
    abnormalcounter1 = abnormalcounter1 - 1;
  } 
else {
    abnormalcounter1 = 640;
  }  

if (abnormalcounter1 <= 0) {
  Serial.println("Device Malfunction");
  String Malfunction = "Device Malfunction";
  charac.writeValue(Malfunction);
}

/////////////IF THERMISTOR WORKING, BUT MICROPHONE ISN'T////////////////  Abnormal Breathing
if ((BPM > 0 || BPM < 100) && (BPMmic == 0 || BPMmic > 100)) { 
    if (BPM > UpperLimit || BPM < LowerLimit) {
      abnormalcounter2 = abnormalcounter2 - 1;
    } 
    else {
      abnormalcounter2 = 640; 
    }  
}
else {
  abnormalcounter2 = 640; // Number of abnormal readings until alert
}

if (abnormalcounter2 <= 0) {
  String bpm_mic = String("!");
  Serial.println("Abnormal Breathing detected");
  String ABD = "Abnormal Breathing Detected!";
  charac.writeValue(ABD);
}

/////////////IF THERMISTOR ISN'T WORKING, BUT MICROPHONE IS////////////// Abnormal Breathing
if ((BPM == 0 || BPM > 100) && (BPMmic > 0 || BPMmic < 100)) {
    if (BPMmic > UpperLimit || BPMmic < LowerLimit) {
      abnormalcounter3 = abnormalcounter2 - 1;
    } 
    else {
      abnormalcounter3 = 640; 
    }  
}
else {
  abnormalcounter3 = 640; // Number of abnormal readings until alert
}

if (abnormalcounter3 <= 0) {
  String bpm_temp = String("!");
  Serial.println("Abnormal Breathing detected");
  String ABD = "Abnormal Breathing Detected!";
  charac.writeValue(ABD); 
}

/////////////         DEVICE DISLODGEMENT - BOTH SENSORS WORKING           ////////////// 10 seconds ---> 12 seconds
if (slope < 0) { 
      abnormalcounter4 = abnormalcounter4 - 1;
}
else {
  abnormalcounter4 = 640; // Number of abnormal readings until alert
}

if (abnormalcounter4 <= 0) {
  Serial.println("Device Dislodgement Detected");
  String Dislodgement = "Device Dislodgement Detected";
  charac.writeValue(Dislodgement);
}


  
  }
  
  }  
record_ready = false;
}
Serial.println("Bluetooth disconnected. Please reconnect device to access TrachAlert readings.");
}
}

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
