# project_ta

7/6/2022 Notes:
- Microphone readings are now displayed alongside IMU readings
- Fixed issue with LED remaining permanently on after viewing the serial plotter
- Added the option for a delay between data measurements
- Created secondary, experimental code aiming to tackle serial plotter issues

7/18/2022 Notes:
- Decreased delay between samples to increase sampling frequency (to 6.25 Hz from 4 Hz)

In order to change the sampling frequency within Edge Impulse, the following command may need to be inputted into the terminal:
```
edge-impulse-data-forwarder --frequency 1
```
where the "1" represents the desired frequency in Hz.

7/20/2022 Notes:
- Added an FFT code, which can run successfully on XIAO BLE Sense after downloading the PDM library (can be found in repository)

