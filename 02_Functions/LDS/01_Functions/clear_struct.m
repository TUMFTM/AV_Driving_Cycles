function [vehicle]=clear_struct(vehicle)
%% Description:
% This function removes all unnecessary information of the vehicle struct

% Author:    Korbinian Moller
% date:      10.05.21
%% Inputs:
% vehicle struct
%% Outputs:
% reduced vehicle struct
%% Sources: 
%% Implementation:
clear_struct = true;

if clear_struct == true
    
    fields_standard = {'t','delta_t','v','a','alpha','resistance','T_wheels','n_wheels','P_wheels','P_batt','traction_limit','E_bat_step','E_bat_cum'};

    if strcmp(vehicle.LDS.settings.drive,'front_wheel')
        fields_front = {'gear_f','overload_vector_mot_f','T_mot_f_gear','n_mot_f_gear','eta_mot_f_gear','T_mot_f','n_mot_f','eta_mot_f','P_mech_mot_f_gears','P_el_mot_f_gears','P_el_mot_f','gear_distribution_f','max_overload_time_in_cycle_mot_f','P_batt_gears'};
        fields = [fields_standard, fields_front];
    elseif strcmp(vehicle.LDS.settings.drive,'rear_wheel')
        fields_rear = {'gear_r','overload_vector_mot_r','T_mot_r_gear','n_mot_r_gear','eta_mot_r_gear','T_mot_r','n_mot_r','eta_mot_r','P_mech_mot_r_gears','P_el_mot_r_gears','P_el_mot_r','gear_distribution_r','max_overload_time_in_cycle_mot_r','P_batt_gears'};
        fields = [fields_standard, fields_rear];
    else
        fields_awd = {'max_overload_time_in_cycle_mot_f', 'overload_vector_mot_f','max_overload_time_in_cycle_mot_r', 'overload_vector_mot_r','eta_mot_f','eta_mot_r','T_mot_f','T_mot_r','n_mot_f','n_mot_r','gear','torque_distribution','torque_distribution_cycle','gear_distribution_f','gear_distribution_r','P_mech_mot_f','P_mech_mot_r','P_el_mot_f','P_el_mot_r'};
        fields = [fields_standard, fields_awd];
    end
    
    
    vehicle.LDS.sim_cons = rmfield(vehicle.LDS.sim_cons, fields);

end
