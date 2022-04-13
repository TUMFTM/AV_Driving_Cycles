%% Description: evaluate generated cycle
% Designed by: Duan, Xucheng (FTM, Technical University of Munich)
%-------------
% Created on: 09.01.2022
% Last update: 11.01.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: function for running AVDC(Autonomous Vehicle Driving Cycle) simulation with certain driving style
% ------------
% Input:    - Logs: logs from generated cycle
%
% ------------
% Output:   - T
%           - v_avg
%           - v_max
%           - a_rms
%           - j_rms
%           - r_TTC
%           - N_ovt
%           - b_100
%
% ------------
% References:   
%%-------------

function Res = eval6DC(Logs)
    v = Logs.v_ego; % [m/s]
    v_min = 1;
    % trim cycle
    ii = v.Length;
    while ii > 0
        if v.Data(ii) >= v_min
            break;
        else
            v = delsample(v, 'Index', ii);
            ii = ii - 1;
        end
    end
    % features
    Res.T = max(v.Time) - min(v.Time); % [s]
    Res.v_avg = mean(v.Data) * 3.6; % [km/h]
    Res.v_max = max(v.Data) * 3.6; % [km/h]
    a = dts(v);
    Res.a_RMS = rms(a.Data); % [m/s^2]
    j = dts(a);
    Res.j_RMS = rms(j.Data); % [m/s^3]
    id = Logs.id_front;
    Res.r_TTC = mean(1 ./ Logs.TTC.Data);
    Res.n_ovt = id.Data(id.Length);
    Cons = Eval_Consumption(Logs);
    Res.b_100 = Cons.consumption100km; % [kWh/100km]
end

function tsout = dts(tsin)
    t = tsin.Time;
    x = tsin.Data;
    dt = diff(t);
    dx = diff(x);
    v = dx ./ dt;
    tsout = timeseries([v;0],t);
end