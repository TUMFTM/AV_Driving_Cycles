%% Description: script to generate AVDC with certain driving style
% Designed by: Duan, Xucheng (FTM, Technical University of Munich)
%-------------
% Created on: 09.01.2022
% Last update: 09.01.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: skript for running AVDC(Autonomous Vehicle Driving Cycle) simulation with certain driving style
% ------------
% Input:    - driving style: Comf,Safe,Fast
%           - traffic cycle: traf_cyc
%
% ------------
% Output:   - driving cycle of autonomous vehicle
% ------------
% References:   
%%-------------
clear variables;

%% input variables
Input.Comf = 6;
Input.Safe = 7;
Input.Fast = 8;
Input.traf_cyc = 'EUROPE_ARTEMIS_URBAN'; % EUROPE_ARTEMIS_URBAN, EUROPE_ARTEMIS_ROAD, EUROPE_ARTEMIS_MOTORWAY
Input.traf_type = 'urban'; %'urban','road','highway','auto';

%% add workspace
try
    cd 'D:\Studium\SS21\Masterarbeit\03_Programm\01_Main'
    run_mode = 'local';
catch
    % Lehrstuhl-Rechner
    cd 'W:\Projekte\Studenten Schockenhoff\21 SS\MA Duan\03_Programm\01_Main'
    run_mode = 'FTM';
end
addpath(genpath('../'))
clear run_mode

%% run simulation
Res = func_AVDC(Input);

%% output
AVDC.cycle = Res.Logs.v_ego; % [m/s]
