%% Description: function for calulating consumption of driving cycle
% Designed by: Duan, Xucheng (FTM, Technical University of Munich)
%-------------
% Created on: 26.11.2021
% Last update: 26.11.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: calculate energiy consumption of driving cycle with LDS from FTM
%              
% ------------
% Input:    - driving cycle
% ------------
% Output:   - consumption
%           
% ------------
% References:   - LDS from FTM
%                     Authors:    Lorenzo Nicoletti, Phd Candidate, Institute of Automotive Technology, Technical University of Munich
%                                Ruben Hefele, Student, Technical University of Munich
%                                Korbinian Moller, Student, Technical University of Munich
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
    tn = (0:0.1:max(tc))';
    vn = interp1(tc,vc,tn,'makima') ./ 3.6;
    Logs.v_ego = timeseries(vn,tn);
    warning('off','MATLAB:dispatcher:UnresolvedFunctionHandle')
    Cons = Eval_Consumption(Logs);
    cons = Cons.consumption100km;
end