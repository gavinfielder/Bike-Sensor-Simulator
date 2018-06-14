%% Control Script
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
% frame tilt [rad]
th_f = @(t) c1*(  0.2*sin(0.0001*t)  );  
% Forward speed
v_f  = @(t) c3*(  1   );    
% Wheel turn angle
T_f  = @(t) c2*( 0 );
                                                % TODO: update to
                                                % handlebar angle
    
% Define program constants
tUpperBound = 200000;
clk_res = 0.001; % Clock resolution
fprintf('Program will calculate first %g units of time.\n', (tUpperBound*clk_res));


% Run program
run absoluteCalculations
run sensorAbsoluteCalculations
run sensorRelativeCalculations


%% Plot Linear Acceleration over time
figure(6); clf;
plot(timeValues(1:tUpperBound-2),lx,'.r'); hold on; grid on;
plot(timeValues(1:tUpperBound-2),ly,'.g'); 
plot(timeValues(1:tUpperBound-2),lz,'.b');
xlabel('Time'); ylabel('Accelerometer Reading');
title('Noiseless accelerometer readings');
legend('x','y','z');

%% Plot angular acceleration over time
figure(7); clf;
plot(timeValues(2:tUpperBound-2),gx(2:tUpperBound-2),'.r'); hold on; grid on;
plot(timeValues(2:tUpperBound-2),gy(2:tUpperBound-2),'.g'); 
plot(timeValues(2:tUpperBound-2),gz(2:tUpperBound-2),'.b');
xlabel('Time'); ylabel('Gyro Reading');
title('Noiseless gyroscope readings');
legend('x','y','z');

%% Take a noisy sample of lx
lx_noisy = noisySample(lx,133,1e-3);
tt = (1:length(lx_noisy))';
figure(8);clf; plot(tt,lx_noisy,'-k'); grid on;

%% Take a noisy sample of gy
gy_noisy = noisySample(gy,133,0.8e-3);
tt = (1:length(gy_noisy))';
figure(9);clf; plot(tt,gy_noisy,'-k'); grid on;

%% Take a noisy sample of ly with the same error parameters as lx
% this will act as our baseline for R
ly_noisy = noisySample(ly,133,1e-3);
tt = (1:length(ly_noisy))';
figure(10);clf; plot(tt,ly_noisy,'-k'); grid on;
                         
%% Take a noisy sample of gz to act as a baseline of gy
gz_noisy = noisySample(gz,133,0.8e-3);
tt = (1:length(gy_noisy))';
figure(11);clf; plot(tt,gz_noisy,'-k'); grid on;