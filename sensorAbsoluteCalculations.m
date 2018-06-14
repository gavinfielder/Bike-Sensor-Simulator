%% Sensor Absolute Calculations

% Calculate absolute sensor position and orientation
disp('Calculating absolute sensor position and orientation...');
% Note coefficients are inverted here because they'll be more useful
% multiplying incoming sensor readings, not generated theoretical reading
sx = (x + c4*h*cos(rho).*sin(th));     % sensor absolute x
sy = (y - c4*h*sin(rho).*sin(th));     % sensor absolute y
sz = (c4*h*cos(th));                   % sensor absolute z
ax = 0;                             % sensor absolute pitch angle
ay = th;                            % sensor absolute roll angle
az = rho;                           % sensor absolute yaw angle

% Determine pitch, yaw, roll (angular velocity) of the sensor
disp('Calculating sensor angular velocity...');
pitch = sin(th).*d_rho./clk_res;
yaw = cos(th).*d_rho./clk_res;
roll = d_th./clk_res;
    
% Done message
disp('Done calculating sensor absolute position.');
