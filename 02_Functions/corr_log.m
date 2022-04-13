%% Description: logarithm correlation
% Designed by: Duan, Xucheng (FTM, Technical University of Munich)
%-------------
% Created on: 04.01.2022
% Last update: 04.01.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: logarithm correlation 5~10 with input at 5 and 10
%              
% ------------
% Input:    - x5, x10: x value for 5 and 10
% ------------
% Output:   - y
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