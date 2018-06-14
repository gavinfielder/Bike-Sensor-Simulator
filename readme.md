# Gavin's Bike Sensor Simulator

Part of the unmanned bicycle project

This program accepts function inputs for:
 - Bike velocity v(t) over time
 - Frame tilt th(t) over time
 - Wheel turn angle T(t) over time

Using these functions and specified parameters, this program calculates
the position of the bike and position of the sensor at discrete time
steps. By taking derivatives and vector projections, it obtains curves
that model the accelerometer and gyroscope readings. 

See Demo.m script for demo program.

## Known Issues

At the time of writing this simulator, I didn't know that gyroscopes measure angular velocity, not angular acceleration. Theoretically, if gyroscopes did measure angular acceleration, this simulator would be accurate, but as it stands, the gyro portion is completely wrong. 

Last updated 11/20/2017
