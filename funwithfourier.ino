#include <PDM.h>                 //Includes the PDM library, which allows the obtainment of microphone readings
#include <arduinoFFT.h>          //Includes the arduinoFFT library, which allows for the FFT

#define SAMPLES 256              //Defines the buffer rate, must be a power of 2
#define SAMPLING_FREQUENCY 16000 //Defines the sampling frequency in Hz, which must be less than 10000 due to Analog to Digital Converter (ADC)
short sampleBuffer[SAMPLES];     //Stores a 16-bit value
volatile int samplesRead;        //Creates the integer samplesRead, and voltaile directs the compiler to load the variable from RAM

unsigned long microseconds;      //Stores a 32-bit value


double vReal[SAMPLES];           //Stores value with 64-bit precision
double vImag[SAMPLES];           //Stores value with 64-bit precision

void onPDMdata(void);            //Defines a function, void means function is not expected to return information to function

const uint8_t amplitude = 100;   //Stores value for amplitude

arduinoFFT FFT = arduinoFFT();   //Defines the arduinoFFT function

void setup() {                   //Beginning of setup code
  Serial.begin(115200);          //Sets the data rate in bits per second (baud) for serial data transmission
   
  while (!Serial) {              //While the setup is occurring...
    ; // wait for serial port to connect. Needed for native USB port only
  }
 
  PDM.onReceive(onPDMdata);                     //callback (onPDMdata): function that is called when new PDM data is ready to be read, returns nothing
  PDM.setBufferSize(SAMPLES);                   //size (SAMPLES): buffer size to use in bytes, returns nothing
  //PDM.setGain(0);                             //gain (0): gain value to use, 0 - 255, defaults to 20 if not specified, returns nothing
  // setup the I2S audio input for the sample rate with 32-bits per sample
  if (!PDM.begin(1, 16000)) {                   //channels (1): the number of channels, 1 for mono, 2 for stereo, sampleRate (16000): the sample rate to use in Hz
    Serial.println("Failed to start PDM!");     //Prints an error message 
    while (1);                                  //Function returns 1 on success and 0 on failure
  }
}

void loop() {                                 //Begins the loop...
  if (samplesRead) {                          
    for (int i = 0; i < SAMPLES; i++) {
      vReal[i] = sampleBuffer[i];
      vImag[i] = 0;
    }
    
    //The Short Term Fourier Transform partially converts from the time domain to the frequency domain.
    //Your time resolution goes from 16000 down to 16000/256, but you still know which frames have a peak and which don't.
    
    //Fast Fourier Transform
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD); //Windowing(double *vData, uint16_t samples, uint8_t windowType, uint8_t dir)
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);                 //Compute(double *vReal, double *vImag, uint16_t samples, uint8_t dir);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);                   //ComplexToMagnitude(double *vReal, double *vImag, uint16_t samples);
    
    //You get 16 samples per millisecond, so each set of SAMPLES that you pass to the FFT represents SAMPLES/16 milliseconds. 
    //The FFT trades time resolution for frequency resolution, so you can only measure the peak length with this 16 millisecond granularity. 
    //And you do that by simply counting the number of FFT frames with a peak.
 
    //Pick Peak Frequency
    double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY); //MajorPeak(double *vD, uint16_t samples, double samplingFrequency);
    
    Serial.println(peak); //Prints that peak frequency to the serial monitor
    
    samplesRead = 0;
   
  }
}

void onPDMdata()
{
  int bytesAvailable = PDM.available();
  PDM.read(sampleBuffer, bytesAvailable);
  samplesRead = bytesAvailable / 2;
}
