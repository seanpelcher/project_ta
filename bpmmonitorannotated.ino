// Libraries
#include "PDM.h"
#include "arduinoFFT.h"
#include "RunningAverage.h"
#include "ArduinoBLE.h"  


#define SAMPLES 256             //Must be a power of 2
#define SAMPLING_FREQUENCY 16000 //Hz, must be less than 10000 due to ADC
short sampleBuffer[SAMPLES];
volatile int samplesRead;

unsigned long microseconds;


double vReal[SAMPLES];
double vImag[SAMPLES];

void onPDMdata(void);

const uint8_t amplitude = 100;

arduinoFFT FFT = arduinoFFT();

RunningAverage myRA(10); //for the running average
int samples = 0;

int UpperThreshold = 2500;
int LowerThreshold = 1700;
int reading = 0;
float BPM = 0.0;
bool IgnoreReading = false;             // IgnoreReading starts false...
bool FirstPulseDetected = false;        // FirstPulseDetected starts false...
unsigned long FirstPulseTime = 0;       // FirstPulseTime starts at 0...
unsigned long SecondPulseTime = 0;      // SecondPulseTime starts at 0...
unsigned long PulseInterval = 0;        // PulseInterval starts at 0...
const unsigned long delayTime = 10;     // delayTime
const unsigned long delayTime2 = 1000;  // delayTime2
const unsigned long baudRate = 9600;
unsigned long previousMillis = 0;       //PreviousMillis starts at 0...
unsigned long previousMillis2 = 0;      //PreviousMillis2 starts at 0...

void setup(){                           // Nothing interesting happens during the setup...
  Serial.begin(baudRate);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
   myRA.clear();
    PDM.onReceive(onPDMdata);
  PDM.setBufferSize(SAMPLES);
  //PDM.setGain(0);
  // setup the I2S audio input for the sample rate with 32-bits per sample
  if (!PDM.begin(1, 16000)) {
    Serial.println("Failed to start PDM!");
    while (1); 
} 
}

void loop(){

 if (samplesRead) {
    for (int i = 0; i < SAMPLES; i++) {
      vReal[i] = sampleBuffer[i];
      vImag[i] = 0;
    }

    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
 
    double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);
    
  //Serial.println(peak);   //prints all frequency values, even background noise and voices
    
    samplesRead = 0;   
    
    double rn = peak;
    myRA.addValue(rn);
    samples++;
    Serial.print("Running Average: ");
    Serial.println(myRA.getAverage(), 4); //prints the running average after each frequency that is printed

    if (samples == 4000)
    {
     samples = 0;
      myRA.clear();
    }
    samplesRead = 0;
     //delay(100);
     
  // Get current time
  unsigned long currentMillis = millis();                   // Sets millis() to be the current time...

  // First event
  if(myTimer1(delayTime, currentMillis) == 1){              // If myTimer1 is equal to 1... [[[

    reading = myRA.getAverage();                            // Set variable "reading" to the current running average...

    // Heart beat leading edge detected.                    // We are trying to detect the start of the "peak" of the breath so...
    if(reading > UpperThreshold && IgnoreReading == false){ // If the current running average is greater than the upper threshold and IgnoreReading is false... {{{
      if(FirstPulseDetected == false){                      // If FirstPulseDetected is false... (this should only happen one time)
        FirstPulseTime = millis();                          // Set FirstPulseTime to the current time...
        FirstPulseDetected = true;                          // And change FirstPulseDetected to be true...
      }
      else{                                                 // Otherwise... (if the reading is not greater than upper threshold or IgnoreReading is true...)
        SecondPulseTime = millis();                         // Set SecondPulseTime to the current time...
        PulseInterval = SecondPulseTime - FirstPulseTime;   // Calculate the PulseInterval by subtracing FirstPulseTime from SecondPulseTime...
        FirstPulseTime = SecondPulseTime;                   // And then set the FirstPulseTime equal to the previous SecondPulseTime... 
      }
      IgnoreReading = true;                                 // Change IgnoreReading to be true...
      digitalWrite(LED_BUILTIN, HIGH);                      // And turn off the LED... }}}
    }

    // Heart beat trailing edge detected.                   // Now we are trying to detect the end of the "peak" of the breath so...
    if(reading < LowerThreshold && IgnoreReading == true){  // If the current running average is less than the lower threshold and IgnoreReading is true... {{{
      IgnoreReading = false;                                // Change IgnoreReading to be false...
      digitalWrite(LED_BUILTIN, LOW);                       // And turn the LED on... }}}
    }  

    // Calculate Beats Per Minute.                          // Now calculate the BPM...
    BPM = (1.0/PulseInterval) * 60.0 * 1000;                // Using the found PulseInterval... ]]]
  }

  // Second event
  if(myTimer2(delayTime2, currentMillis) == 1){             // If myTimer2  is equal to 1... [[[
    Serial.print(reading);                                  // Print the current running average...
    Serial.print("\t");
    Serial.print(PulseInterval);                            // And the pulse interval...
    Serial.print("\t");
    Serial.print(BPM);                                      // And the BPM...
    Serial.println(" BPM");
    Serial.flush();
  }
}
}

// First event timer
int myTimer1(long delayTime, long currentMillis){
  if(currentMillis - previousMillis >= delayTime){previousMillis = currentMillis;return 1;}
  else{return 0;}
}

// First event timer explained...
// If the current time subtracted by the previous stored time (1) is greater than or equal to delayTime (1)...
// Then set the previous stored time (1) to the current time and set myTimer1 equal to 1...
// Otehrwise, set myTimer1 equal to 0...

// Second event timer
int myTimer2(long delayTime2, long currentMillis){
  if(currentMillis - previousMillis2 >= delayTime2){previousMillis2 = currentMillis;return 1;}
  else{return 0;}
}

// Second event timer explained...
// If the current time subtracted by the previous stored time (2) is greater than or equal to delayTime (2)...
// Then set the previous stored time (2) to the current time and set myTimer2 equal to 1...
// Otehrwise, set myTimer2 equal to 0...

void onPDMdata() // Not relevant, only necessary for FFT to function...
{
  int bytesAvailable = PDM.available();
  PDM.read(sampleBuffer, bytesAvailable);
  samplesRead = bytesAvailable / 2;

}
