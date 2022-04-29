%% Description: linear correlation
% Designed by: Duan, Xucheng (FTM, Technical University of Munich)
%-------------
% Created on: 04.01.2022
% Last update: 05.01.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: Linear correlation 0~10 with input at 5 and 10.
%              The values for output 5 and 10 are given as x5 and x10.
%              A linear correlation line is defined with these values.
%              The input x will be scaled to score y.
%
% ------------
% Input:    - x5, x10: x value for 5 and 10
%           - x: value to be scaled
% ------------
% Output:   - y: scaled value [5,10]
% ------------
% References: 
%
%%-------------

function y = corr_lin(x, x5, x10)
    y = (x - x5) / (x10 - x5) * 5 + 5;
end