%% Description: logarithm correlation
% Designed by: Duan, Xucheng (FTM, Technical University of Munich)
%-------------
% Created on: 04.01.2022
% Last update: 05.01.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: Logarithm correlation 5~10 with input at 5 and 10.
%              The values for output 5 and 10 are given as x5 and x10.
%              A logarithm correlation curve is defined with these values.
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

function y = corr_log(x, x5, x10)
    % y = b * ln(a * x)
    a = x10 / x5 ^ 2;
    b = 5 / (log(x10 / x5));
    y = b * log(a * x);
end