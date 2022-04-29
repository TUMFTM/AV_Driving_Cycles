%% Description: frequency weighting according to ISO 2631-1
% Designed by: Duan, Xucheng (FTM, Technical University of Munich)
%-------------
% Created on: 01.09.2021
% Last update: 27.01.2022
% ------------
% Version: Matlab2020b, Control System Toolbox
%-------------
% Description: frequency weighting W_d and W_f from to ISO 2631-1,
%              
% ------------
% Input:    - a: accerlation array of the vehicle [m/s^2]
%           - t: time corresponding of a [s]
%           - mode: the frequency weightings to be used
%                   'd': using weighting W_d, for comfort in x&y direction
%                   'f': using weighting W_f, for motion sickness
% ------------
% Output:   - aw: frequency weighted acceleration [m/s^2]
%           
% ------------
% References:   - [1] ISO 2631-1:1997
%               - [2] ISO 2631-1:1997/AMD 1:2010
%%-------------

function [aw] = FrequencyWeighting2631(a, t, mode)
    % ISO 2631-1:1997, Annex A
    if strcmp(mode, 'd')
        f1 = 0.4;
        f2 = 100;
        f3 = 2;
        f4 = 2;
        Q4 = 0.63;        
    elseif strcmp(mode, 'f')
        f1 = 0.08;
        f2 = 0.63;
        f3 = inf;
        f4 = 0.25;
        Q4 = 0.86;
        f5 = 0.0625;
        Q5 = 0.8;
        f6 = 0.1;
        Q6 = 0.8;
    else
        error('Invalid frequency weighting mode!')
    end
    omega1 = 2 * pi * f1;
    H_h = tf([1,0,0],[1, sqrt(2) * omega1, omega1^2]);
    omega2 = 2 * pi * f2;
    H_l = tf([1],[1 / omega2^2, sqrt(2) / omega2, 1]);
    omega3 = 2 * pi * f3;
    omega4 = 2 * pi * f4;
    H_t = tf([1 / omega3, 1],[1 / omega4^2, 1 / (Q4 * omega4), 1]);
    if strcmp(mode, 'd')
        H_s = tf([1],[1]);
    else
        omega5 = 2 * pi * f5;
        omega6 = 2 * pi * f6;
        H_s = tf([1, omega5 / Q5, omega5^2],[1, omega6 / Q6, omega6^2]);
    end
    H = series(H_h, H_l);
    H = series(H, H_t);
    H = series(H, H_s);
    % bode(H);
    % interpolate to at least 1Hz
    if max(diff(t))>1
        ti = (floor(min(t)):1:ceil(max(t))).';
        ai = interp1(t, a, ti, 'makima');
        t = ti;
        a = ai;
    end
    % response of transfer function
    aw = lsim(H,a,t);
end