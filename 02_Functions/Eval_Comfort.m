%% Description: function for calulating comfort indicators
% Designed by: Duan, Xucheng (FTM, Technical University of Munich)
%-------------
% Created on: 14.09.2021
% Last update: 22.11.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: calculate comfort indicators with speed profile
%              
% ------------
% Input:    - Logs: log struct from Simulink
%           
% ------------
% Output:   - res: struct of comfort indicators
%           
% ------------
% References:   - ISO 2631-1
%%-------------
function res = Eval_Comfort(Logs)
    t = Logs.v_ego.Time;
    v = Logs.v_ego.Data;
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
    % frequency weighting from ISO 2631
    a_comf = FrequencyWeighting2631(a, t - min(t), 'd');
    res.a_comf_RMS = rms(a_comf);
    a_sick = FrequencyWeighting2631(a, t - min(t), 'f');
    res.a_sick_RMS = rms(a_sick);
    res.j_RMS = rms(j);
    res.MSDV = sqrt(trapz(t, a_sick .* a_sick)); % motion sickness dose value, see ISO 2631-1:1997 Annex D
    % scaling with normal distribution function
    res.F_a_comf = cdf('Normal',res.a_comf_RMS,0.117696,0.0590699);
    res.F_a_sick = cdf('Normal',res.a_sick_RMS,0.336494,0.216002);
    res.F_j = cdf('Normal',res.j_RMS,0.843757,0.388959);
    % combining 3 indicators
    res.CF = (1 - sqrt(res.F_a_comf^2 + res.F_a_sick^2 + res.F_j^2) / sqrt(3)) * 6 + 4;
end