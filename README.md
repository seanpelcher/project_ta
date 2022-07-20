# project_ta

7/6/2022 Notes:
Created the project_ta repository!

7/7/2022 Notes:
Microphone readings are now displayed alongside IMU readings
Fixed issue with LED remaining permanently on after viewing the serial plotter
Added the option for a delay between data measurements
Created secondary, experimental code aiming to tackle serial plotter issues

7/18/2022 Notes:
- Decreased delay between samples to increase sampling frequency (to 6.25 Hz from 4 Hz)

In order to change the sampling frequency within Edge Impulse, the following command may need to be inputted into the terminal:
```
edge-impulse-data-forwarder --frequency 1
```
where the "1" represents the desired frequency in Hz.

7/20/2022 Notes:
- Added a basic FFT code (found [here](https://1littleendian.medium.com/the-late-night-tinkering-projects-10-fun-with-fourier-a72b358229b3)) to the repository

This code can run on the Seeed XIAO BLE Sense nRF52840 board, but requires two additional libraries. <br />
The download for the first library, the arduinoFFT library, can be found [here](https://www.arduino.cc/reference/en/libraries/arduinofft/). <br />
The download for the second library, the PDM library, can be found as a .zip file in the project_ta repository.

