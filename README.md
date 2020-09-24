# Shimmer Tools
Tools for working with the Shimmer3 wearable devices (not affiliated with the company who makes them)


### Shimmer3 Calibration Verifier (AHRS Visualizer)
A tool for checking the calibration of a device (or just visualize the orientation)
  - Built on Python 3.7 (64-bit) in Windows 10 (Likely to work just fine under Linux with the depricated RFCOMM tool)
  - Requires pygame (obtainable through pip) and a Shimmer3 device configured for Wide-range acclerometer, gyroscope and magnetometer at 51.2 Hz
  - Will use the calibration settings stored on the device
  - It uses orientation.py which is licensed under GNU_Lesser_GPL
  - There is a math related error that occurs every now and then (likely to be related to the update frequency of the model and the sampling frequency difference)

Replace COM5 with the port your devices uses.

### ToDo
  - Figure out the math problem in the AHRS Visualizer
  - Multiple sensor visualizations in one window
  - A HAR (Human Activity Recognition) tool will be uploaded in the near future

> **Created for the research profile [Embedded Sensor System for Health](https://www.es.mdh.se/projects/324-ESS_H___Embedded_Sensor_Systems_for_Health_Research_Profile) at [Mälardalen University](https://www.mdh.se/en/malardalen-university?), Sweden.**
