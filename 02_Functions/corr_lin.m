%% Description: linear correlation
% Designed by: Duan, Xucheng (FTM, Technical University of Munich)
%-------------
% Created on: 22.11.2021
% Last update: 22.11.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: linear correlation 0~10 with input at 5 and 10
%              
% ------------
% Input:    - x5, x10: x value for 5 and 10
% ------------
% Output:   - y
% ------------
% References: 
%
%%-------------

function y = corr_lin(x, x5, x10)
    y = (x - x5) / (x10 - x5) * 5 + 5;
end