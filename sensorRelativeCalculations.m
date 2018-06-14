%% Sensor-basis (Relative) Calculations

%% Declare memory
% These arrays use derivatives, so have fewer points
vx = zeros(tUpperBound-1,1);
vy = zeros(tUpperBound-1,1);
vz = zeros(tUpperBound-1,1);

%% Relative calculations

% Determine absolute component velocity - note row vector
disp('Calculating absolute velocity vectors...');
d_s = [ (sx(2:tUpperBound) - sx(1:tUpperBound-1))./clk_res, ...
        (sy(2:tUpperBound) - sy(1:tUpperBound-1))./clk_res, ...
        (sz(2:tUpperBound) - sz(1:tUpperBound-1))./clk_res   ];
% d_s = [ (sx(2:tUpperBound) - sx(1:tUpperBound-1)), ...
%         (sy(2:tUpperBound) - sy(1:tUpperBound-1)), ...
%         (sz(2:tUpperBound) - sz(1:tUpperBound-1))   ];

% Translate ax,ay,az into normalized vectors for dot product usage
disp('Translating sensor orientation to normalized vector form...');
sensorForward = [sin(rho), cos(rho), zeros(tUpperBound,1)];
sensorRight = [cos(rho).*cos(th), -sin(rho).*cos(th), sin(th)];
sensorUp = cross(sensorRight,sensorForward,2);

% Calculate component velocity using sensor basis
disp('Calculating component velocity using sensor basis...');
for t=1:tUpperBound-1
    vx(t) = dot(d_s(t,:),sensorRight(t,:)); % component of velocity in right dir
    vy(t) = dot(d_s(t,:),sensorForward(t,:)); % comp of vel in sensor forward dir
    vz(t) = dot(d_s(t,:),sensorUp(t,:)); % comp of vel in sensor up dir
end

% Finally, calculate simulated sensor readings
%
% These are the simulated sensor readings, so we'll add in the coefficients
% Note coefficients are inverted because it'll be useful later to use them
% to multiply *incoming* data, not for generating simulated data.
%
% Calculate linear acceleration of sensor
disp('Generating accelerometer readings...');
lx = (1/c7)*(vx(2:tUpperBound-1)-vx(1:tUpperBound-2))./clk_res;
ly = (1/c8)*(vy(2:tUpperBound-1)-vy(1:tUpperBound-2))./clk_res;
lz = (1/c15)*(vz(2:tUpperBound-1)-vz(1:tUpperBound-2))./clk_res;
% Use angular velocity of the sensor to generate gyrometer readings
disp('Generating gyrometer readings...');
gx = (1/c9)*(pitch(2:tUpperBound-1)-pitch(1:tUpperBound-2))./clk_res;
gy = (1/c10)*(roll(2:tUpperBound-1)-roll(1:tUpperBound-2))./clk_res;
gz = (1/c11)*(yaw(2:tUpperBound-1)-yaw(1:tUpperBound-2))./clk_res;

% Let user know calculation complete
disp('Done.');