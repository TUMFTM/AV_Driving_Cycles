%% Description: running ACC simulation with parameters
% Designed by: Duan, Xucheng (FTM, Technical University of Munich)
%-------------
% Created on: 20.08.2021
% Last update: 11.10.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: running ACC simulation with parameters
% ------------
% Input:    - Para: Struct of parameters for the simulation and ACC-controller
%
% ------------
% Output:   - simOut: output from Simulink
%           - Eval: Struct of properties for evaluation
% ------------
% References:   - [1] Skript - VL Fahrerassistanzsystem
%               - [2] H. Winner, S. Hakuli, F. Lotz, und C. Singer, 
%                     Handbuch Fahrerassistenzsysteme: Grundlagen, Komponenten und Systeme für aktive Sicherheit und Komfort, 
%                     3rd Aufl. Wiesbaden: Springer Vieweg, 2015.
%                     Chap. 46 - Adaptive Cruise Control
%%-------------
clear variables;
addpath(genpath('D:\Studium\SS21\Masterarbeit\03_Programm'))
addpath(genpath('../'))

savename = 'reference_ARRoad'; % name for saving workspace

%% load default parameters
cycle_front = 'EUROPE_ARTEMIS_ROAD';
Para = init_ACC_Sim(cycle_front);

%% override parameters
% Para.speed_set = 120;
% Para.t_set	 = 1.5;
% Para.d0      = 4;
% Para.P_a_x   = [-10,0,10];  % [m/s]
% Para.P_a_y   = [-15,0,15];    % [m/s^2]
% Para.P_v_x   = [-100,0,100];  % [m]
% Para.P_v_y   = [-15,0,15];      % [m/s]
% Para.P_a_x   = [-10,-2,0,2,10];  % [m/s]
% Para.P_a_y   = [-7,-1,0,1,7];    % [m/s^2]


%% run simulation model
simdl = 'Cascade_ACC_Para'; 
open_system(simdl)
mdlWks = get_param(simdl,'ModelWorkspace');
Para_list = fieldnames(Para);
for ii = 1:length(Para_list)
    assignin(mdlWks,Para_list{ii},Para.(Para_list{ii}))
end
simOut = sim(simdl);

%% results evalutation
% load logged signals
log_list = simOut.logsout.getElementNames;
Logs = struct;
for ii = 1:length(log_list)
    Logs.(log_list{ii}) = simOut.logsout{ii}.Values;
end
% trim log to end of the trip
for ii = length(Logs.v_ego.Data):-1:1
    if Logs.v_ego.Data(ii) > 0.1
        i_end = ii;
        break
    end
end
for ii = 1:length(log_list)
    Logs.(log_list{ii}) = ...
        delsample(Logs.(log_list{ii}),'Index',i_end:Logs.(log_list{ii}).Length);
end
Logs.v_rel = Logs.v_front - Logs.v_ego;
clear log_list i_end t_end
clear ii

if Logs.AEB_triggered.data(end) > 0
    warning('AEB is triggered during the simulation!')
end

%% evalutation
% Eval.a_max = max(Logs.a_ego);   % max accelerlation [m/s^2]
% Eval.a_min = min(Logs.a_ego);   % max deceleration [m/s^2]
% Eval.a_RMS = rms(Logs.a_ego.Data);  % RMS accerlation
% Eval.j_max = max(Logs.jerk);    % max positive jerk [m/s^3]
% Eval.j_min = min(Logs.jerk);    % max negative jerk [m/s^3]
% Eval.j_RMS = rms(Logs.jerk.Data);   % RMS jerk
% Eval.a_comf = FrequencyWeighting2631(Logs.a_ego.Data, Logs.a_ego.Time, 'd');
% Eval.a_comf_RMS = rms(Eval.a_comf); % frequency weighted RMS accerlation for comfort
% Eval.a_sick = FrequencyWeighting2631(Logs.a_ego.Data, Logs.a_ego.Time, 'f');
% Eval.a_sick_RMS = rms(Eval.a_sick); % frequency weighted RMS accerlation for motion sickness
% Eval.MSDV = sqrt(trapz(Logs.a_ego.Time, Eval.a_sick .* Eval.a_sick));
% %Eval.x_rel_min = min(Logs.x_rel);  % min distance [m]
% Eval.t_rel_min = min(Logs.x_rel ./ Logs.v_ego); % min time headway [s]
% Eval.TTC_min = min(Logs.TTC);   % min Time-to-Collision [s]
% Eval

%% Evaluation V2
%warning('off','MATLAB:dispatcher:UnresolvedFunctionHandle')
Comf = Eval_Comfort(Logs)
Cons = Eval_Consumption(Logs)
Safe = Eval_Safety(Logs)

% save mat
save(['./ACC_Data/',savename,'_',datestr(now,'yymmdd')])


function Para = init_ACC_Sim(cycle_front)
%% load default parameters (do not change)
    Para.speed_set = 120;         % ACC set speed [km/h]
    Para.Range_Sensor = 200;      % Sensor detection range [m]
    Para.t_sensor = 0.2;          % Sensor time delay [s]
    Para.t_set	 = 2;             % time headway [s]
    Para.d0      = 4;             % constant distance [m]
    Para.P_a_x   = [-10,0,10];    % [m/s]
    Para.P_a_y   = [-7,0,7];      % [m/s^2]
    Para.P_v_x   = [-100,0,100];  % [m]
    Para.P_v_y   = [-7,0,7];      % [m/s]
    Para.a_max_x = [0,5,20,50];   % [m/s]
    Para.a_max_y = [4,4, 2, 2];     % [m/s^2]
    Para.a_min_x = [-1,0,1e-4, 5,  20,  50]; % [m/s]
    Para.a_min_y = [ 0,0,  -5,-5,-3.5,-3.5]; % [m/s^2]
    Para.j_min_x = [ 0, 5,  20,  50];   % [m/s]
    Para.j_min_y = [-5,-5,-2.5,-2.5];     % [m/s^3]
% load driving cycle of front vehicle
    load([cycle_front,'.mat'])
    if exist('dc','var')
        Para.cycle_t = dc.time;
        Para.cycle_v = dc.speed;
    else
        Para.cycle_t = t;
        Para.cycle_v = v;
    end
end