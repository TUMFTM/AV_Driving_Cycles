%% Description: function for calulating consumption of driving cycle
% Designed by: Duan, Xucheng (FTM, Technical University of Munich)
%-------------
% Created on: 26.11.2021
% Last update: 26.11.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: Calculate energiy consumption of driving cycle with LDS from FTM
%              
% ------------
% Input:    - cycle: driving cycle
% ------------
% Output:   - cons: energy consumption [kWh/100km]
%           
% ------------
% References:   - An Open-Source Modular Quasi-Static Longitudinal Simulation for Full Electric Vehicles
%                 Authors: König, Adrian; Nicoletti, Lorenzo;et al.
%                 doi: 10.1109/EVER48776.2020.9242981
%%-------------

function cons = DC_Cons(cycle) % 'EUROPE_ARTEMIS_URBAN','EUROPE_ARTEMIS_ROAD','EUROPE_ARTEMIS_MOTORWAY'
    load([cycle,'.mat'])
    if exist('dc','var')
        tc = dc.time;
        vc = dc.speed * 3.6;
    else
        tc = t;
        vc = v;
    end
    if min(tc) > 0
        tc = [0;tc];
        vc = [0;vc];
    end
    tn = (0:0.1:max(tc))'; % interpolate to 10 Hz
    vn = interp1(tc,vc,tn,'makima') ./ 3.6;
    Logs.v_ego = timeseries(vn,tn);
    warning('off','MATLAB:dispatcher:UnresolvedFunctionHandle')
    Cons = Eval_Consumption(Logs);
    cons = Cons.consumption100km;
end