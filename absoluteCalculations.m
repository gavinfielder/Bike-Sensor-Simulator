%% Absolute Calculations
%% Memory Declaration

% Declare memory
disp('Declaring Memory...');
timeValues = zeros(tUpperBound,1);
th = zeros(tUpperBound,1);
v = zeros(tUpperBound,1);
T = zeros(tUpperBound,1);
x = zeros(tUpperBound,1);
y = zeros(tUpperBound,1);
rho = zeros(tUpperBound,1);
d_rho = zeros(tUpperBound,1);
ec = zeros(tUpperBound,1);
vy_sensor = zeros(tUpperBound,1);

%% World-Basis (Absolute) Calculations
                  
% Generate data for state functions - speeds later calculations
disp('Generating data for state functions...');
for t = 1:tUpperBound
    th(t) = th_f(t);
    v(t) = v_f(t);
    T(t) = T_f(t);
end

% First derivative of state functions
disp('Calculating first derivative of state functions...');
d_th = th(2:tUpperBound) - th(1:tUpperBound-1);
d_v = v(2:tUpperBound) - v(1:tUpperBound-1);
d_T = T(2:tUpperBound) - T(1:tUpperBound-1);

% Store initial values
disp('Storing initial values of absolute state...');
x(1) = x_0;
y(1) = y_0;
rho(1) = rho_0;
timeValues(1) = 0;

% Iterate for absolute position and direction
disp('Calculating absolute state...');
for t = 2:tUpperBound
    timeValues(t) = (t-1) * clk_res;
    % Get new rho and position deltas from function call
    [d_x, d_y, rho(t), d_rho(t)] = getDeltas(T(t),v(t),rho(t-1),L,d,clk_res);
    x(t) = x(t-1) + d_x; % get new x position of bicycle
    y(t) = y(t-1) + d_y; % get new y position of bicycle
end

% Scale absolute position and direction
disp('Scaling absolute state...');
x = c4 * x;
y = c4 * y;
rho = c6 * rho;

% Done message
disp('Done calculating bike absolute position.');











