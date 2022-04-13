%% Description: function for running AVDC simulation with parameters
% Designed by: Duan, Xucheng (FTM, Technical University of Munich)
%-------------
% Created on: 04.11.2021
% Last update: 04.11.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: function for running AVDC(Autonomous Vehicle Driving Cycle) simulation with parameters
% ------------
% Input:    - Scenario: Struct of driving scenario 
%                    -.cycle: standard driving cycle for traffic
%                    -.type: type of the road ('urban','road','highway')
%                    -.ovt: setting for overtaking ('always','never','auto')
%           - Paraset: Parameter set for simulation
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

function Res = Sim_AVDC(Scenario,Paraset)
    %% load default parameters
    if isfield(Scenario,'cycle')
        cycle_traffic = Scenario.cycle;
        if isfield(Scenario,'type')
            type = Scenario.type;
        else
            type = 'auto';
            warning('Sim_AVDC: road type not defined, using AUTO detection.')
        end
    else
        if isfield(Scenario,'type')
            type = Scenario.type;
        else
            error('Sim_AVDC: road type not defined!')
        end
        switch type
            case 'urban'
                cycle_traffic = 'EUROPE_ARTEMIS_URBAN';
            case 'road'
                cycle_traffic = 'EUROPE_ARTEMIS_ROAD';
            case 'highway'
                cycle_traffic = 'EUROPE_ARTEMIS_MOTORWAY';
            otherwise
                error('Sim_AVDC: Invalid input for road type!')
        end
    end
    if isfield(Scenario,'ovt')
        ovt = Scenario.ovt;
    else
        ovt = 'auto';
    end
    % preprocess driving cycle
    cycle = prepare_cycle(cycle_traffic, type, ovt);
    Para = init_AVDC_Sim(cycle);
    clear type ovt
    
    %% override parameters
    Paraset_list = fieldnames(Paraset);
    for ii = 1:length(Paraset_list) % inject parameters
        Para.(Paraset_list{ii}) = Paraset.(Paraset_list{ii});
    end
    clear Paraset_list ii
    
    %% run simulation model
    simdl = 'AV_homo_traffic';
    load_system(simdl)
    mdlWks = get_param(simdl,'ModelWorkspace');
    Para_list = fieldnames(Para);
    for ii = 1:length(Para_list) % inject parameters
        assignin(mdlWks,Para_list{ii},Para.(Para_list{ii}))
    end
    clear Para_list
    simOut = sim(simdl);
    
    %% results evalutation
    % load logged signals
    log_list = simOut.logsout.getElementNames;
    log = cell(log_list);
    for ii = 1:length(log_list)
        log{ii} = simOut.logsout{ii}.Values;
    end
    Logs = tscollection(log,'Name','AVDC_Sim');
    Logs = addts(Logs, Logs.v_front - Logs.v_ego, 'v_rel');
    clear log_list log ii

    %% Evaluation
    warning('off','MATLAB:dispatcher:UnresolvedFunctionHandle')
    Res.Comf = Eval_Comfort(Logs);
    Res.Cons = Eval_Consumption(Logs);
    Res.Safe = Eval_Safety(Logs);
    Res.Fast = Eval_Speed(Logs,cycle);
    Res.Logs = Logs;
end

%% Function: prepare driving cycle
function cycle = prepare_cycle(cycle_traf, type, ovt)
    try
        load([cycle_traf,'_',type,'_',ovt,'.mat'],'cycle')
        % disp('Loading prepared cycle...')
    catch
        disp('Prepared cycle not found, processing standard cycle...')
        load([cycle_traf,'.mat'])
        if exist('dc','var')
            tc = dc.time;
            vc = dc.speed * 3.6;
        else
            tc = t;
            vc = v;
        end
        cycle = Preprocess_DC(tc, vc, type, ovt);
        try
            save(['03_Simulation\Prepared_Cycle\',cycle_traf,'_',type,'_',ovt,'.mat'],'cycle')
        catch
            
        end
    end
end

%% Function: load default parameters (do not change)
function Para = init_AVDC_Sim(cycle)
% ACC
    Para.Range_Sensor = 250;      % Sensor detection range [m]
    Para.t_sensor = 0.2;          % Sensor time delay [s]
    Para.t_set	 = 2;             % time headway [s]
    Para.d0      = 2;             % constant distance [m]
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
    Para.set_speeds = [50, 100, 150];   % target speed for urban, road and highway [km/h]
% Overtake
    Para.ovt_tol = 20;            % lower limit of relative speed for overtake [km/h]
% Traffic
    Para.t_delta_traf = 10;       % time difference between trafic vehicles [s]
% Driving Cycle of traffic
    Para.cycle_t = cycle.t;
    Para.cycle_v = cycle.v;
    Para.cycle_x = cycle.x;
    Para.cycle_type = cycle.type;
    Para.cycle_dovt = cycle.dovt;
    Para.cycle_xx = cycle.x;
    ii = 2;
    while ii <= length(Para.cycle_xx)
        if Para.cycle_xx(ii) <= Para.cycle_xx(ii-1)
            Para.cycle_type(ii-1) = min(Para.cycle_type(ii-1), Para.cycle_type(ii));
            Para.cycle_dovt(ii-1) = min(Para.cycle_dovt(ii-1), Para.cycle_dovt(ii));
            Para.cycle_xx(ii) = [];
            Para.cycle_type(ii) = [];
            Para.cycle_dovt(ii) = [];
        else
            ii = ii + 1;
        end
    end
end