%% Description: function to generate AVDC with certain driving style
% Designed by: Duan, Xucheng (FTM, Technical University of Munich)
%-------------
% Created on: 09.01.2022
% Last update: 24.01.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: function for running AVDC(Autonomous Vehicle Driving Cycle) simulation 
%              with a certain driving style.
% ------------
% Input:    - Input: structure of inputs, with 
%                  .Comf/Safe/Fast: driving style aspects (6 to 10) 
%                  .traf_cyc: name of traffic cycle
%                  .type: road type for traffic cycle
%
% ------------
% Output:   - Res: structure of simulation results
%                .
% ------------
% References:   
%%-------------

function Res = func_AVDC(Input)
    %% check driving style
    load('Cali7tab.mat') % load calibrated parameter table
    Cali = Cali7tab;
    % reset rating 10
    for ii = 1:size(Cali,1)
        for jj = 9:11 % column for comf,safe,fast
            if(Cali.(jj)(ii) >= 9.5) % rating 10s are reduce to solve parameters
                Cali.(jj)(ii) = 10; % reset to 10
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
    Paraset = Get_Paraset(Cali{pos_para,[1:8]}); % load parameters from the table
    clear ii jj

    %% run simulation
    Scen.cycle = Input.traf_cyc;
    Scen.type = Input.traf_type;
    Scen.ovt = 'auto';
    Res = Sim_AVDC(Scen,Paraset); % run simulation
end