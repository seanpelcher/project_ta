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
BLEDevice central = BLE.central();                                // a bluetooth connected is awaited

if(central) {                                                     // the central loop starts once bluetooth is connected
  Serial.println("Bluetooth connection established.");            // tell the user that the connection is established...
  Serial.print("Connected to central: ");                         // and state the address of the connected device
  Serial.println(central.address());

while(central.connected()){                                       // the connected loop starts only if there is a bluetooth connection
if(gate > 2){                                                     // the gate was opened during setup so this loop is entered
  Serial.println("////////// TrachAlert System //////////");      
  String title = "TrachAlert System";
  charac.writeValue(title);
  Serial.println("Manual Mode (1) or Age Mode (2)?");             // ask the user if they would like to enter manual or age mode
  String manual = "Manual Mode (1) or Age Mode (2)";              // manual mode allows the user to input their own upper and lower limits
  charac.writeValue(manual);                                      // age mode simply requires an age input from the user and automatically sets limits
  
  while (central.connected() && middletunnel > 2) {               // if bluetooth is connected and the middle tunnel is open (which it should be)...
    if(charac.written()){                                         // if the user inputted either a "1" or "2" for the mode...
      ModeInput = charac.value();                                 // set the mode to either manual for "1" or age "2"...
      Mode = ModeInput.toInt();                                   // and convert the mode into an integer...
      middletunnel = 1;                                           // then close the middle tunnel...
    }
  }                                                               // so that this loop can't be accessed again    

  if (Mode == 1) {                                                                // if the mode is set to manual...
    Serial.println("What should the upper limit for the respiratory rate be?");   // ask what the upper limit should be...
  String firstquestion = "Input Upper Limit: ";
  charac.writeValue(firstquestion);
  while (central.connected() && uppertunnel > 2) {                             // if bluetooth is connected and the upper tunnel is open (which it should be)...
    if(charac.written()){                                                      // if the user inputted an upper limit...
      UpperLimitInput = charac.value();                                        // set the upper limit to that number...
      UpperLimit = UpperLimitInput.toInt();                                    // and convert the upper limit into an integer...
      uppertunnel = 1;                                                         // then close the upper tunnel...                    
    }
  }                                                                            // so that this loop can't be accessed again

  Serial.print("Upper respiratory rate limit set to: ");                       // tell the user what the upper limit was set to
  String firstresponse = "Upper Limit set to: ";
  charac.writeValue(firstresponse);
  Serial.print(UpperLimit);
  UpperLimitW = String(UpperLimit);
  charac.writeValue(UpperLimitW);
  Serial.println(" BPM");

  Serial.println("And what should the lower limit for the respiratory rate be?"); // ask what the lower limit should be...
  String secondquestion = "Input Lower Limit: ";
  charac.writeValue(secondquestion);
  while (central.connected() && lowertunnel > 2) {                                // if bluetooth is connected and the lower tunnel is open (which it should be)...
    if(charac.written()){                                                         // if the user inputted a lower limit...
      LowerLimitInput = charac.value();                                           // set the lower limit to that number...
      LowerLimit = LowerLimitInput.toInt();                                       // and convert the lower limit into an integer...
      lowertunnel = 1;                                                            // then close the lower tunnel...
    }
  }                                                                               // so that this loop can't be accessed again
  
  Serial.print("Lower respiratory rate limit set to: ");                          // tell the user what the lower limit was set to
  String secondresponse = "Lower Limit set to: ";
  charac.writeValue(secondresponse);
  Serial.print(LowerLimit);
  LowerLimitW = String(LowerLimit);
  charac.writeValue(LowerLimitW);
  Serial.println(" BPM");
  
  Serial.println("Thank you for choosing TrachAlert. Respiratory rate measurement will start in 5 seconds."); // thank the user and prepare to start measurements
  String thanks = "Thank you.";
  charac.writeValue(thanks);
  counter = 1000;                    // initial counter value
  caution = 300;                     // initial caution value
  escape = 0;                        // initial escape value;
  delay(5000);                       // seconds before respiratory rate measurement begins
  gate = 1;                          // close the gate so that the manual mode loop cannot be entered again
  Mode = 0;                          // set the mode to 0 as an extra precaution to avoid entering the loop again
  }                                  // exit manual mode setup loop

  if (Mode == 2) {                                                                                    // if the mode is set to age...
  Serial.println("What is the age of the patient? (if patient is less than 1 year old, input 0)");    // ask what the age of the patient is...
  String firstquestion = "Input Age in years:";
  charac.writeValue(firstquestion);
  while (central.connected() && uppertunnel > 2) {           // if bluetooth is connected and the upper tunnel is open (which it should be)...
    if(charac.written()){                                    // if the user inputted an age...
      AgeInput = charac.value();                             // set the age to that number...
      Age = AgeInput.toInt();                                // and convert the age into an integer...
      uppertunnel = 1;                                       // then close the upper tunnel...
    }
  }                                                          // so that this loop can't be accessed again
    
  Serial.print("Age set to ");                               // tell the user what the age was set to
  String firstresponse = "Age set to: ";
  charac.writeValue(firstresponse);
  Serial.print(Age);
  AgeW = String(Age);
  charac.writeValue(AgeW);
  Serial.println(" yrs");
  
  if (Age < 1) {                // if the age was set to less than 1 (infant, etc.)
    UpperLimit = 60;            // set the upper limit to 60...
    LowerLimit = 30;            // and the lower limit to 30
  }
  if (Age >= 1 && Age < 3){     // if the age was set to greater than or equal to 1 or less than 3
    UpperLimit = 40;            // set the upper limit to 40...
    LowerLimit = 24;            // and the lower limit to 24
  }
  if (Age >= 3 && Age < 6){     // if the age was set to greater than or equal to 3 or less than 6
    UpperLimit = 34;            // set the upper limit to 34...
    LowerLimit = 22;            // and the lower limit to 22
  }
  if (Age >= 6 && Age < 12){    // if the age was set to greater than or equal to 6 or less than 12
    UpperLimit = 30;            // set the upper limit to 30...
    LowerLimit = 18;            // and the lower limit to 18
  }
  if (Age >= 12 && Age < 18){   // if the age was set to greater than or equal to 12 or less than 18
    UpperLimit = 16;            // set the upper limit to 16...
    LowerLimit = 12;            // and the lower limit to 12
  }
  if (Age >= 18 && Age < 200){  // if the age was set to greater than or equal to 18 or less than 200
    UpperLimit = 18;            // set the upper limit to 18...
    LowerLimit = 12;            // and the lower limit to 12
  }
  
  Serial.print("Lower respiratory rate limit set to: ");  // tell the user what the lower limit was set to
  String secondresponse = "Lower Limit set to: ";
  charac.writeValue(secondresponse);
  Serial.print(LowerLimit);
  LowerLimitW = String(LowerLimit);
  charac.writeValue(LowerLimitW);
  Serial.println(" BPM");

delay (500);  // wait 0.5 seconds...

  Serial.print("Upper respiratory rate limit set to: ");  // tell the user what the upper limit was set to
  String thirdresponse = "Upper Limit set to: ";
  charac.writeValue(thirdresponse);
  Serial.print(UpperLimit);
  UpperLimitW = String(UpperLimit);
  charac.writeValue(UpperLimitW);
  Serial.println(" BPM");
  
  Serial.println("Thank you for choosing TrachAlert. Respiratory rate measurement will start in 5 seconds.");  // thank the user and prepare to start measurements
  //String thanks = "Thank you.";
  //charac.writeValue(thanks);
  counter = 1000;     // initial counter value
  caution = 300;      // initial caution value
  escape = 0;         // initial escape value;
  delay(5000);        // seconds before respiratory rate measurement begins
  gate = 1;           // close the gate so that the manual mode loop cannot be entered again
  Mode = 0;           // set the mode to 0 as an extra precaution to avoid entering the loop again
  }                   // exit age mode setup loop...
}                     // and exit the gate loop as well (still in the while command)

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
Serial.print(";    ");
Serial.print(BPMmic);                               // the BPM as determined by the microphone...
Serial.print(" BPM Mic");
Serial.print(";    ");
Serial.print(BPM);                                  // and the BPM as determined by the thermistor...
Serial.println(" BPM Temp");
numberbpm = BPM;
wordbpm = String(numberbpm);
numberbpmmic = BPMmic;
wordbpmmic = String(numberbpmmic);
String bpm_mic = String(wordbpmmic + " BPM Mic   ");
String bpm_temp = String(wordbpm + " BPM Temp");
String phonedisplay = String(bpm_mic + "; " + bpm_temp);
charac.writeValue(phonedisplay);                          // and will relay this information to the connected bluetooth device!

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
// Check if microphone AND thermistor readings exceed upper limit or drop below lower limit
if (BPM > UpperLimit || BPM < LowerLimit) {             // if the BPM from the thermistor is greater than the upper limit or less than the lower limit...
  if (BPMmic > UpperLimit || BPMmic < LowerLimit) {     // if the BPM from the microphone is greater than the upper limit or less than the lower limit...
    abnormalcounter = abnormalcounter - 1;              // subtract 1 from the abnormal breathing counter
  } 
  else { abnormalcounter = 640; }                     // otherwise, return the abnormal breathing counter to 640
}
  
else { abnormalcounter = 640; }                       // otherwise, return the abnormal breathing counter to 640

if (abnormalcounter <= 0) {                       // if the abnormal breathing counter is less than or equal to 0 (from 640 consecutive abnormal readings)...
  Serial.println("Abnormal Breathing detected");  // send the user an ABNORMAL BREATHING ALERT
  String ABD = "Abnormal Breathing Detected!";
  charac.writeValue(ABD);
}

// If both microphone and thermistor yield zero readings for 10 consecutive seconds (NO BREATHING OR DEVICE MALFUNCTION)
if ((BPM == 0 || BPM > 100) && (BPMmic == 0 || BPMmic > 100)) {  // if the BPMs from the thermistor and microphone are equal to 0 or greater than 100
    abnormalcounter1 = abnormalcounter1 - 1;                     // subtract 1 from the (1) abnormal breathing counter
  } else { abnormalcounter1 = 640; }                             // otherwise, return the (1) abnormal breathing counter to 640

if (abnormalcounter1 <= 0) {                  // if the (1) abnormal breathing counter is less than or equal to 0...
  Serial.println("Device Malfunction");       // send the user a DEVICE MALFUNCTION ALERT
  String Malfunction = "Device Malfunction";
  charac.writeValue(Malfunction);
}

// If the thermistor is working and the microphone is not (ABNORMAL BREATHING)
if ((BPM > 0 || BPM < 100) && (BPMmic == 0 || BPMmic > 100)) {  // if the BPM from the thermistor is between 0 and 100 but the BPM from the microphone is equal to 0 or greater than 100
    if (BPM > UpperLimit || BPM < LowerLimit) {                 // if the BPM from the thermistor is greater than the upper limit or less than the lower limit
      abnormalcounter2 = abnormalcounter2 - 1;                  // subtract 1 from the (2) abnormal breathing counter
    } 
    else { abnormalcounter2 = 640; }                           // otherwise, return the (2) abnormal breathing counter to 640
}
  
else { abnormalcounter2 = 640; }                               // otherwise, return the (2) abnormal breathing counter to 640  

if (abnormalcounter2 <= 0) {                      // if the (2) abnormal breathing counter is less than or equal to 0...
  String bpm_mic = String("!");                   // set the microphone BPM string to "!"
  Serial.println("Abnormal Breathing detected");  // and send the user an ABNORMAL BREATHING ALERT
  String ABD = "Abnormal Breathing Detected!";
  charac.writeValue(ABD);
}

// If the thermistor is not working and the microphone is (ABNORMAL BREATHING)
if ((BPM == 0 || BPM > 100) && (BPMmic > 0 || BPMmic < 100)) {  // if the BPM from the microphone is between 0 and 100 but the BPM from the thermistor is equal to 0 or greater than 100
    if (BPMmic > UpperLimit || BPMmic < LowerLimit) {           // if the BPM from the microphone is greater than the upper limit or less than the lower limit
      abnormalcounter3 = abnormalcounter3 - 1;                  // subtract 1 from the (3) abnormal breathing counter
    } else { abnormalcounter3 = 640; }                          // otherwise, return the (3) abnormal breathing counter to 640
}
  
else { abnormalcounter3 = 640; }                                // otherwise, return the (3) abnormal breathing counter to 640

if (abnormalcounter3 <= 0) {                      // if the (3) abnormal breathing counter is less than or equal to 0...
  String bpm_temp = String("!");                  // set the thermistor BPM string to "!"
  Serial.println("Abnormal Breathing detected");  // and send the user an ABNORMAL BREATHING ALERT
  String ABD = "Abnormal Breathing Detected!";
  charac.writeValue(ABD); 
}

// If both sensors are working but the device has been dislodged
if (slope < 0) {                                 // if the slope of the thermistor readings is less than 0
      abnormalcounter4 = abnormalcounter4 - 1;   // subtract 1 from the (4) abnormal breathing counter
} else { abnormalcounter4 = 640; }               // otherwise, return the (4) abnormal breathing counter to 640

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
