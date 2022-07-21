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
Added a basic FFT code (found [here](https://1littleendian.medium.com/the-late-night-tinkering-projects-10-fun-with-fourier-a72b358229b3)) to the repository, under funwithfourier.ino. <br/>
<br />
This code can run on the Seeed XIAO BLE Sense nRF52840 board, but requires two additional libraries. <br />
The download for the first library, the arduinoFFT library, can be found [here](https://www.arduino.cc/reference/en/libraries/arduinofft/). <br />
The download for the second library, the PDM library, can be found as a .zip file in the project_ta repository.

