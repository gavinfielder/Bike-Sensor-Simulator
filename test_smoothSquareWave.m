%% Test smooth square wave
clear;clc;

k = 100;
% t = (1:2000)';

for t = 1:2000
    y(t) = smoothSquareWave(t,k);  
end

t = (1:2000)';
figure(1); clf;
plot(t,y,'.b');
ylim([-2, 2]);