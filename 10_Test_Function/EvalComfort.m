%% Description: function for calulating comfort indicators
% Designed by: Duan, Xucheng (FTM, Technical University of Munich)
%-------------
% Created on: 14.09.2021
% Last update: 14.09.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: calculate comfort indicators with speed profile
%              with selected comfort indicators
% ------------
% Input:    - t: array of time [s]
%           - v: array of speed [m/s]
% ------------
% Output:   - res: struct of comfort indicators
%           
% ------------
% References:   - ISO-2631
%%-------------
function res = EvalComfort(t, v)
    res = struct();
    t = t - min(t);
    % interpolate to at least 1Hz
    if max(diff(t))>0.1
        ti = (floor(min(t)):0.1:ceil(max(t))).';
        vi = interp1(t, v, ti, 'makima');
        t = ti;
        v = vi;
    end
    dt = [diff(t); 1]; 
    dv = [diff(v); 0];
    a = dv ./ dt;
    da = [diff(a); 0];
    j = da ./ dt;
    a_comf = FrequencyWeighting2631(a, t - min(t), 'd');
    res.a_comf_RMS = rms(a_comf);
    a_sick = FrequencyWeighting2631(a, t - min(t), 'f');
    res.a_sick_RMS = rms(a_sick);
    res.j_RMS = rms(j);
    res.MSDV = sqrt(trapz(t, a_sick .* a_sick));
end