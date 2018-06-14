function [d_x, d_y, rho, d_rho] = getDeltas(T, v, rho, L, d, clk_res)
%getDeltas  Returns change in position
%   @param  T  angle of handlebars - positive is right turn
%   @param  v  current speed
%   @param  rho  current direction angle
%   @param  k  velocity coefficient
%   @param  L  distance between front wheel and rear wheel
%   @param  clk_res  clock resolution coefficient
%
%   @return  d_x  change in x-coordinate
%   @return  d_y  change in y-coordinate
%   @return  rho  updated direction of bicycle
%   @return  d_rho  the change in direction
%
%   See model documentation for derivations

    % Find change in direction of bicycle
    d_rho = (v ./ sqrt(d^2 + (L / tan(T)).^2)) * clk_res;
    if T < 0
        d_rho = d_rho * (-1);
    end
    
    % Find new direction of bicycle
    rho = rho + d_rho;
    
    % Find delta x, delta y
    d_x = (v .* sin(rho)) * clk_res;
    d_y = (v .* cos(rho)) * clk_res;

end

