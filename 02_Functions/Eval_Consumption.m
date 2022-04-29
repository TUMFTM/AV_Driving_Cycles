%% Description: function for calulating consumption
% Designed by: Duan, Xucheng (FTM, Technical University of Munich)
%-------------
% Created on: 21.09.2021
% Last update: 04.10.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: calculate energiy consumption with LDS from FTM
%              
% ------------
% Input:    - t: array of time [s]
%           - v: array of speed [m/s]
% ------------
% Output:   - res: struct of consumption indicators
%           
% ------------
% References:   - An Open-Source Modular Quasi-Static Longitudinal Simulation for Full Electric Vehicles
%                 Authors: König, Adrian; Nicoletti, Lorenzo;et al.
%                 doi: 10.1109/EVER48776.2020.9242981
%%-------------
function res = Eval_Consumption(Logs)
    t = Logs.v_ego.Time;
    v = Logs.v_ego.Data;
    res = struct();
    %% IMPORTANT: Here the User can assign the Inputs and tune the Fixed Parameter (constant value used for the simulation)!
    [vehicle,Parameters] = initialize_inputparameters_model3rwd();

    %% IMPROTANT: here the user can decide wheter to output or not the warning signs of the longitudinal simulation
    vehicle.LDS.settings.suppress_LDS_warnings=1; %Set to 1 to suppress the LDS warnings and to 0 to show them

    %% Load cycle
    % using given t,v instead of standard driving cycle
    %vehicle = load_cycle(vehicle,Parameters);
    vehicle = load_profile(vehicle, t, v);

    %% Fill the topology dependent values, fill values from the inputs:
    vehicle=initialize_topology(vehicle,Parameters); 

    %% Calculate missing inputs (dependent on the given Inputs): n_max, gear ratio 
    vehicle=calc_missing_inputs(vehicle,Parameters);

    %% Load and scale Motor characteristics
    vehicle.LDS = load_engine(vehicle.LDS);

    %% Acceleation simulation:
    vehicle = acceleration_sim(vehicle,Parameters);

    %% Max speed simulation:
    vehicle=max_speed_sim(vehicle,Parameters);

    %% Energy consumption simulation
    vehicle = energy_consumption_sim(vehicle, Parameters);  %Calculation of energy consumption
    
    %% Prepare results
    res.E_bat = vehicle.LDS.sim_cons.E_bat_sum; % consumed energy [kWh]
    res.consumption100km = vehicle.LDS.sim_cons.consumption100km; % energy consumption [kWh/100km]
end


function [veh] = load_profile(veh, t, v)
%% Description
% loads speed profile
%author:    Lorenzo Nicoletti, Korbinian Moller, FTM, TUM
%date:      16.07.2020
% modified by Duan, Xucheng
% date:     21.09.2021
% Input:    - t: array of time [s]
%           - v: array of speed [m/s]
%% Output:
%   veh:    updated struct with the vehicle parameters
%% 1) Load profile:

%Load speed profile:
dc.t = t;
dc.v = v;
dc.S = zeros(size(dc.v));

while dc.t(1) < 0 %timesteps <0 are cut off
    dc.t(1,:) = [];
    dc.v(1,:) = [];
    dc.S(1,:) = [];
end   

if dc.t(1) ~= 0 %cycle has to start at timestep 0                              
    dc.t = dc.t - dc.t(1);
end

%Assign the timestep in s
delta_t = dc.t(2)-dc.t(1); %no delta_t input available
% 
% %create t vector
% t = (0:delta_t:dc.t(end))';
t = dc.t;
% 
% %Assign the cycle speed in m/s
% v = dc.v/3.6;

%create Interpolation for speed
F_v = griddedInterpolant(dc.t,v,'linear','none');

%Assign interpolated cycle speed in m/s
v = F_v(t);
                    
%Assign the cycle acceleration in  m/s�
a = [0; diff(v)]./delta_t;  

%Assign slope S. Check if a vector S containing the elevation in % for each timestep is assigned
if isfield(dc,'S')
    alpha=dc.S;
    F_S = griddedInterpolant(dc.t,alpha,'linear','none');
    alpha = F_S(t);
else
    %No vector S in MATLAB cycle variable, the elevation will be taken as 0% for all steps
    alpha   = zeros(size(v));                                       
end  

%% 2) Assign output
%As the cycle has already been loaded, the values are saved, so, that it does not have to be loaded again
veh.LDS.sim_cons.t          = t;                                            %time vector in s
veh.LDS.sim_cons.delta_t    = delta_t;                                      %time step in s
veh.LDS.sim_cons.v          = v;                                            %speed vector in m/s
veh.LDS.sim_cons.a          = a;                                            %acceleration vector in m/s^2 
veh.LDS.sim_cons.alpha      = alpha;                                        %slope vector in %
        
end