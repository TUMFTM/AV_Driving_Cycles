function [vehicle] = calc_longitudinal_simulation(vehicle,Parameters)

%%----------------------------------------------------Characteristic Path----------------------------------------------------
%looks like: '02_Characteristics_Diagrams'
Parameters.LDS.char_path = '02_Characteristics_Diagrams'; %Path where engine characteristics are stored within the folder /Mainfolder/03_LDS_Hefele/XXXXX

%%----------------------------------------Parameters for the simulation settings----------------------------------------
Parameters.LDS.max_diff_to_acc          =   0.1;                            %tolerance of acceleration time in s
Parameters.LDS.max_diff_to_max_speed    =   2;                              %tolerance of actual speed to max speed in km/h
Parameters.LDS.t_sim                    =   25;                             %maximum acceleration simulation time in s
Parameters.LDS.v_max_sim                =   100;                            %max speed in acceleration simulation in km/h
Parameters.LDS.delta_t                  =   0.1;                            %step size in acceleration simulation in s
Parameters.LDS.delta_t_con              =   0.1;                            %step size in consumption simulation in s
Parameters.LDS.axle_load_front          =   0.5;                                %axis load distribution on front axis 
Parameters.LDS.axle_load_rear           =   1-Parameters.LDS.axle_load_front;   %axis load distribution on rear axis 
Parameters.LDS.slope_angle_sim_acc      =   0;                              %Slope angle for acceleration simulation in °

%----------------------------------------------------------
%This is a temporary state, this variables have to be unified (use everywhere the same name)
%vehicle.Input.vehicle_sim_cons_weight      = NaN;              
vehicle.masses.vehicle_sim_cons_weight = NaN; %vehicle weight for energy consumption simulation (optional, otherwise NaN)
vehicle.Input.weight_extra_equipment=vehicle.Input.extra_equipement; 
vehicle.Input.number_passengers=vehicle.Input.number_of_seats;
vehicle.LDS.settings.suppress_LDS_warnings=vehicle.settings.suppress_warnings; %Set to 1 to suppress the LDS warnings and to 0 to show them
%----------------------------------------------------------

% Adjust Torque ratio to satisfy requirements (just for AWD)
if count(vehicle.Input.topology,'G')==2
    if vehicle.Input.torque_ratio<0.5
        vehicle.Input.T_max_Mot_f=1;
        vehicle.Input.T_max_Mot_r=(1/vehicle.Input.torque_ratio)-1;
    else
        vehicle.Input.T_max_Mot_r=1;
        vehicle.Input.T_max_Mot_f=vehicle.Input.torque_ratio/(1-vehicle.Input.torque_ratio);
    end
end

%% Load cycle
vehicle = load_cycle(vehicle,Parameters);

%% Fill the topology dependent values, fill values from the inputs:
vehicle=initialize_topology(vehicle,Parameters); 

%% Calculate missing inputs (dependent on the given Inputs): n_max, gear ratio 
vehicle=calc_missing_inputs(vehicle,Parameters);

%% Load and scale Motor characteristics
vehicle.LDS = load_engine(vehicle.LDS);

%% Acceleation simulation
vehicle = acceleration_sim(vehicle,Parameters);

%% Max speed simulation
vehicle=max_speed_sim(vehicle,Parameters);

%% Energy consumption simulation
vehicle = energy_consumption_sim(vehicle, Parameters);  

%% gradeability calculation
vehicle = calc_gradeability(vehicle, Parameters);

%% clear unused data
vehicle = clear_struct(vehicle);

end
