%% Demo Script for Gavin's Bike Simulator
% This script creates three plots: a 3D plot of the bicycle position
% and the sensor location, the corresponding linear acceleration as should
% be read through the accelerometer, and the corresponding angular
% acceleration as should be read through the gyroscope.
%
% This simulator might be of use when testing mathematical models, or
% simply for generating theoretical data to use with sensor fusion or
% filtering algorithms
%
% Dependencies: these files should be in the working directory:
%   1. absoluteCalculations.m
%   2. sensorAbsoluteCalculations.m
%   3. sensorRelativeCalculations.m
%   4. getDeltas.m

clear;clc;

disp('Defining Constants...');

% Define physical system constants
h = 2; % height from ground to sensor
L = 8; % Distance between back wheel and front wheel
d = 1; % Lateral distance between back wheel and sensor location

% Set Initial physical system values
x_0     = 0; % bike initial position x
y_0     = 0; % bike initial position y
rho_0   = 0; % bike initial direction [rad]

% Physical state coefficients
c1 = 1; % Frame tilt physical state coefficient
c2 = 1; % Wheel turn angle physical state coefficient
c3 = 1; % Speed physical state coefficient

% Absolute position and direction scaling
c4 = 1; % Bike absolute position spatial scaling (formerly x alone)
% c5 = 1; % Bike absolute position axis scale y
% c14 = 1; % Bike absolute position axis scale z
c6 = 1; % Bike absolute direction (rho) scaling

% Sensor coefficients
c7 = 1;  % Coefficient of sensor accelerometer x
c8 = 1;  % Coefficient of sensor accelerometer y
c15 = 1; % Coefficient of sensor accelerometer z
c9 = 1;  % Coefficient of sensor gyrometer x (pitch)
c10 = 1; % Coefficient of sensor gyrometer y (roll)
c11 = 1; % Coefficient of sensor gyrometer z (yaw)
c12 = 1; % Coefficient of encoder reading
c13 = 1; % Coefficient of speedometer reading

% Define input state functions
th_f = @(t) c1*(  0.3   );                      % frame tilt [rad]
v_f  = @(t) c3*(  0.3   );                      % Forward speed
freq = @(t) 0.000023*t;
T_f  = @(t) c2*(  0.9*nthroot(sin(freq(t)),1)  );    % Wheel turn angle
    
% Define program constants
tUpperBound = 200000;
clk_res = 0.001; % Clock resolution
fprintf('Program will calculate first %g units of time.\n', (tUpperBound*clk_res));


% Run program
run absoluteCalculations
run sensorAbsoluteCalculations
run sensorRelativeCalculations


% Plot Position
figure(1);clf; hold on;
plot3(x,y,zeros(tUpperBound,1),'.b','MarkerSize',5);
xrange = max(x) - min(x);
yrange = max(y) - min(y);
xlim([min(x)-xrange*0.1, max(x)+xrange*0.1]); 
ylim([min(y)-yrange*0.1, max(y)+yrange*0.1]); 
% Plot sensor absolute position
plot3(sx,sy,sz,'.r','MarkerSize',5);
% Dress plot
title('Path of bicycle (3D Plot: Use tools to rotate)'); xlabel('x'); ylabel('y');
axis equal; grid on;
legend('Path of bicycle','Path of sensor');

% Plot theoretical accelerometer data
figure(6); clf;
plot(timeValues(1:tUpperBound-2),lx,'.r'); hold on; grid on;
plot(timeValues(1:tUpperBound-2),ly,'.g'); 
plot(timeValues(1:tUpperBound-2),lz,'.b');
xlabel('Time'); ylabel('Accelerometer Reading');
title('Noiseless accelerometer readings (unscaled)');
legend('x','y','z');

% Plot theoretical gyrometer data
figure(7); clf;
plot(timeValues(2:tUpperBound-2),gx(2:tUpperBound-2),'.r'); hold on; grid on;
plot(timeValues(2:tUpperBound-2),gy(2:tUpperBound-2),'.g'); 
plot(timeValues(2:tUpperBound-2),gz(2:tUpperBound-2),'.b');
xlabel('Time'); ylabel('Gyro Reading');
title('Noiseless gyroscope readings (unscaled)');
legend('x','y','z');





                         