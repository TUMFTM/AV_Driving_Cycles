function [veh] = calc_missing_inputs(veh,Par)
%% Description:
%Aim of the function is to identifiy the case and calculate missing parameters.

%author:    Lorenzo Nicoletti, Korbinian Moller
%date:      28.05.2020
%% Inputs:
%vehicle struct: n_max, i_gear, T_max (motor), v_max, acc_time_req
%Parameters struct: needed to be sent to other function
%% Outputs:
%vehicle struct with calculated missing values
%% Implementation:
% 1) Define local variables required for the case identification
% 2) Calculate missing values

%% 1) Define local variables required for the case identification:
%axles that are filled: [front_axle, rear_axle]
filled_axles = veh.LDS.settings.filled_axles;
correction_gearbox=Par.LDS.correction_gearbox;
       
%% 2) Calculate missing values

for i=find(filled_axles)
    
    i_gea=veh.LDS.GEARBOX{i}.i_gearbox;
       
    if ~isnan(i_gea) %gear ratio is an input for all motors
            %Inputs: gear ratio, max_speed.  Wheteher acc_time or T_max or both are given. n_max is not given.

            %calculate max rotational speed of wheels in 1/min
            n_max_wheel = (veh.LDS.sim_acc.max_speed/3.6)/(2*pi*veh.LDS.wheel.r_dyn*1e-3)*60;                               

            %calculate max rotational speed of motors in 1/min
            veh.LDS.MOTOR{i}.n_max = max(veh.LDS.GEARBOX{i}.i_gearbox*n_max_wheel)*((correction_gearbox)^-1); %gear with highest gear ratio determines n_max
    else

        disp('ERROR: You have to give a gearbox trasmission ratio');
        return

    end
    
end

end

