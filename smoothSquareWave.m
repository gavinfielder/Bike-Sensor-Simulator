function [ y ] = smoothSquareWave(t, k)
%smoothSquare Generates a C1 version of a square wave 
% @param  t  discrete time (function input)
% @param  k  pulse width modifier (period = 6k)
% @return y  function value

    i = floor(t/k);
    if mod(i+1,3)==0 %on an edge
        y = cos((pi/k)*(mod((i-2)/3,2)*k + mod(t,k)));
    else
        y = (-1).^(floor(i/3));
    end

end

