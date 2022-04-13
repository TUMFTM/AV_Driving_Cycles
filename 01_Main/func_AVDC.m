%% Description: function to generate AVDC with certain driving style
% Designed by: Duan, Xucheng (FTM, Technical University of Munich)
%-------------
% Created on: 09.01.2022
% Last update: 11.01.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: function for running AVDC(Autonomous Vehicle Driving Cycle) simulation with certain driving style
% ------------
% Input:    - driving style: Comf,Safe,Fast
%           - traffic cycle: traf_cyc,type
%
% ------------
% Output:   - simulation results
% ------------
% References:   
%%-------------

function Res = func_AVDC(Input)
    %% check driving style
    load('Cali7tab.mat')
    Cali = Cali7tab;
    % reset 10s
    for ii = 1:size(Cali,1)
        for jj = 9:11
            if(Cali.(jj)(ii) >= 9.5)
                Cali.(jj)(ii) = 10;
            end
        end
    end
    clear Cali7tab
    % search for desired style
    pos_para = 0;
    for ii = 1:size(Cali,1)
        if Cali.Var_Comf(ii) == Input.Comf
            if Cali.Var_Safe(ii) == Input.Safe
                if Cali.Var_Fast(ii) == Input.Fast
                    pos_para = ii;
                    break;
                end
            end
        end
    end
    if pos_para == 0
        error('Input driving style not valid! Try another combination.')
    end
    Paraset = Get_Paraset(Cali{pos_para,[1:8]});
    clear ii jj

    %% run simulation
    Scen.cycle = Input.traf_cyc;
    Scen.type = Input.traf_type;
    Scen.ovt = 'auto';
    Res = Sim_AVDC(Scen,Paraset);
end