function [traction_limit] = calc_traction_lim(vehicle, Par, F_L , alpha)
%% Description:
%this function determines the max. acceleration due to the traction limit

% Author:   Lorenzo Nicoletti,Ruben Hefele, FTM, TUM
% Date:     26.12.19
% updated:  31.05.2021 (Korbinian Moller)
%% Inputs:
%   vehicle:    struct with the vehicle parameters
%   Par:        struct with fix parameters
%   F_L:        air resistance
%   alpha:      slope angle
%% Outputs:
%  traction_limit: vector containing the max. acceleration at each timestep of the acceleration sim
%% Sources: 
%Haken - Grundlagen der Kraftfahrzeugtechnik
%% Implementation

%Initialize variables to make the equation shorter:
m=vehicle.weights.vehicle_empty_weight_EU;   %Vehicle mass in kg (empty weight + driver)
g=Par.LDS.g;                                 %gravitational acceleration in m/s^2
mu=Par.LDS.mue_max;                          %driving traction coefficient
h_COG=vehicle.parameters.height_COG;         %height COG in mm
wb=vehicle.parameters.wheelbase;             %wheelbase in mm
c_r=vehicle.parameters.c_r;                  %roll resistance coefficient
%load_f=Par.LDS.axle_load_front;               %load distribution front in %
%load_r=Par.LDS.axle_load_rear;                %load distribution rear in %

% calculate traction limit for the different drive types:
if strcmp(vehicle.settings.drive,'rear_wheel')
    load_f = Par.masses.loads.axle_load_front.RWD / 100;
    load_r = 1 - load_f;
    traction_limit = (g*(mu*(load_f*cosd(alpha)+(h_COG/wb)*sind(alpha))-c_r*(load_r*cosd(alpha)-(h_COG/wb)*sind(alpha))-sind(alpha))-(F_L/m))/(1-(h_COG/wb)*(mu+c_r)); 

elseif strcmp(vehicle.settings.drive,'front_wheel')
    load_f = Par.masses.loads.axle_load_front.FWD / 100;
    load_r = 1 - load_f;
    traction_limit = (g*(mu*(load_r*cosd(alpha)-(h_COG/wb)*sind(alpha))-c_r*(load_f*cosd(alpha)+(h_COG/wb)*sind(alpha))-sind(alpha))-(F_L/m))/(1+(h_COG/wb)*(mu+c_r));

elseif strcmp(vehicle.settings.drive,'all_wheel')
    
    traction_limit = g*(mu*cosd(alpha)-sind(alpha))-(F_L/m); 

end

