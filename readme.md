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

