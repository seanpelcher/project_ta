# project_ta

7/6/2022 Notes: <br />
Created the project_ta repository!<br />
We are working with the Seeed XIAO BLE Sense nRF52840 board. <br />
Utilizing two unique sample codes, we are able to gather IMU and microphone readings, but not simultaneously. <br />
Basic information about getting started with the Seeed XIAO BLE Sense nRF52840 board, including the sample codes used to obtain the IMU and microphone readings, can be found [here](https://github.com/kevinwlu/iot/tree/master/lesson6/xiao). <br />
<br />
7/7/2022 Notes: <br />
Microphone readings can now be displayed alongside IMU readings. The code for this can be found under tabbase.ino. <br />
Fixed an issue with the LED light remaining permanently on after viewing the serial plotter. <br />
Added an option for a delay between data measurements. <br />
Created two additional codes, tabedgeimpulse.ino and tabserialplotter.ino, to allow for the IMU and microphone readings to be sent to Edge Impulse and viewed on the serial plotter, respectively. <br/>
<br />
7/18/2022 Notes: <br />
Decreased the delay between samples to increase sampling frequency (to 6.25 Hz from 4 Hz). <br />
<br />
In order to change the sampling frequency within Edge Impulse, the following command may need to be inputted into the terminal:
```
edge-impulse-data-forwarder --frequency 1
```
where the "1" represents the desired frequency in Hz. <br />
<br />
7/20/2022 Notes: <br />
Added a basic FFT code (from [this article](https://1littleendian.medium.com/the-late-night-tinkering-projects-10-fun-with-fourier-a72b358229b3)) to the repository, under funwithfourier.ino. <br/>
<br />
This code can run on the Seeed XIAO BLE Sense nRF52840 board, but requires two additional libraries. <br />
The download for the first library, the arduinoFFT library, can be found [here](https://www.arduino.cc/reference/en/libraries/arduinofft/), and the documentation under the README [here](https://github.com/kosme/arduinoFFT). <br />
The download for the second library, the PDM library, can be found as a .zip file in the project_ta repository, and the documentation [here](https://docs.arduino.cc/learn/built-in-libraries/pdm). <br />
<br />
7/21/2022 Notes: <br />
Fixed an issue with the LED light blinking on and off during the running of the funwithfourier.ino code. <br />
<br />
7/25/2022 Notes: <br />
Updated funwithfourier.ino to include detailed descriptions for most lines of code. <br />
Added a basic bluetooth code to the repository, under tabluetooth.ino. <br />
<br />
8/4/2022 Notes: <br />
Added an important new code, tabbpmlimits.ino, to the repository. <br />
tabbpmlimits.ino is a modification of the code found [here](https://github.com/oelsayed10/ArduinoFFT/blob/main/XiaoMicOriginal.ino). <br />
The original code utilizes the same logic for obtaining heart rate from an EKG reading in order to detect the respiratory rate (in breaths per minute, or BPM) based on frequency measurements. <br />
The new code improves upon the original code by allowing the user to select both an upper and lower limit for the respiratory rate utilizing a basic prompt interface within the serial monitor. Then, if the detected respiratory rate exceeds the upper limit or drops below the lower limit while the code is running, a message appears on the serial monitor alerting the user that an abnormal breathing pattern has been detected. This is accomplished through the use of logic statements and a counter. <br />
<br />
An annotated portion of the code is presented below:
```
if (BPM > UpperLimit || BPM < LowerLimit){   // If the BPM exceeds the inputted upper limit or drops below the inputted lower limit...
    counter = counter - 1;                   // Subtract 1 from the current counter value.
  } else {                                   // Otherwise....
    counter = 300;                           // Reset the counter value to 300...
    digitalWrite(LED_BUILTIN, HIGH);         // And turn the LED off.
  }
  if (counter <= 0) {                               // If the counter value reaches 0 or below...
    digitalWrite(LED_BUILTIN, LOW);                 // Turn the LED on...
    Serial.println("Abnormal Breathing Detected!"); // And alert the doctor of the abnormal breathing pattern.
  }
```
The initial counter value (which is set to a default of 1000 during the void setup of tabbpmlimits.ino) determines how many BPM values should be allowed to pass after measurements begin before an alert for abnormal reading can be printed. The initial counter is important because it could take up to 30 seconds for the code to pick up an accurate respiratory rate, and without a brief buffer period, the abnormal reading alert would likely begin too soon. Once the respiratory rate is within the desired range, however, that value is shortened down (here to 300) in order to allow for a quicker alert if a problem were to occur. These counter values can be changed as needed, and it should be noted that about 32 BPM readings are printed every second. <br />
<br />
8/8/2022 Notes: <br />
Added a variation of tabbpmlimits.ino, called tabbpmlimitsloop.ino, to the repository. The latter retains all the functionality of the former, with the only major difference being that tabbpmlimits.ino prompts the user to input the respiratory rate during void setup and tabbpmlimitsloop.ino prompts the user during void loop (not visible to user). The latter also includes extra lines of dialog to designate the start and end of void setup (visible to user). <br />
Added a variation of tabbpmlimitsloop.ino, called tabbpmlimitsloopbluetooth.ino, to the repository, laying the groundwork for bluetooth integration with the device in the near future.
