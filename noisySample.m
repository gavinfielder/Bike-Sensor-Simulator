function [ output ] = noisySample( input, step, amp )
%noisySample Samples a data set at specified intervals and adds noise
% @param  input  the input data. Assumed no noise
% @param  step   the interval to sample at
% @param  amp    the amplitude of random noise
% @return output the noisy data set

    output = zeros(floor(length(input)/step),1);
    i = 1; j =1;
    while (i < length(input))
        offset = (rand()*2*amp)^0.95-amp;
        output(j) = input(i) + offset;
        i = i + step;
        j = j + 1;
    end

end

