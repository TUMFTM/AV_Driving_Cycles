function vehicle = energy_consumption_sim(vehicle, Par)
%% Description:
%this function determines the vehicle's energy consumption according to the
%chosen driving cycle
%author:    Lorenzo Nicoeltti, Ruben Hefele, FTM, TUM 
%date:      30.12.2019
%% Input:
%parameter struct
%driving cycle
%% Outputs:
%energy consumption in drving cycle
%distance in driving cycle
%velocity in dc
%acceleration
%battery power
%% Implementation:
%1) Initialize varibales from driving cycle
%2) Calculate resistance
%3) Calculate Power requirements, according to the topology
%4) Check traction limit
%5) Assign outputs

%% 1) Initialize varibales from drivin cycle:                                                 

%assign time, timestep, velocity and acceleration 
delta_t =     vehicle.LDS.sim_cons.delta_t;     %time step in s
v       =     vehicle.LDS.sim_cons.v;           %speed vector in m/s
a       =     vehicle.LDS.sim_cons.a;           %acceleration vector in m/s^2 
alpha   =     vehicle.LDS.sim_cons.alpha;       %slope vector in %

%% 2) Calculate resistance:

%Calculate resistance according to loaded cycle:
[vehicle.LDS.sim_cons.resistance, vehicle.LDS.sim_cons.T_wheels, vehicle.LDS.sim_cons.n_wheels]=calc_resistance(v, a, alpha, vehicle.LDS, Par.LDS); 

%% 3) Calculate Power requirements, according to the topology:

%Needed Power at the wheels in W
vehicle.LDS.sim_cons.P_wheels = vehicle.LDS.sim_cons.T_wheels.*vehicle.LDS.sim_cons.n_wheels*2*pi/60;    

check_torque=0;%Marker, needed for the folowing while loop
marker=0; %marker, needed for errorlog, 0 if motor is not rescaled, otherwise 1.

%If the motor torque is too small for the cycle, it has to be scaled
while check_torque==0
    
    %Calculate power, torque, rotational speed of the motor
    calc_power=str2func(['calc_power_', vehicle.LDS.settings.drive]);
    vehicle=calc_power(vehicle, Par);
    
    %Check if the motor torque is sufficient for the cycle
    [check_torque,vehicle]=check_torque_sim_cons(vehicle);
    
    if check_torque==0 %Motor torque is not sufficient and has been scaled
        
        marker=1; 
        
    end

end

if marker==1
    
    %Repeat acceleration sim with new torque-> results in lower acc time
    vehicle = calc_acceleration(vehicle,Par);
    vehicle = max_speed_sim(vehicle,Par);
    
    %Inform the User, that the original torue input has been overwritten
    vehicle.LDS=errorlog(vehicle.LDS,'The input torque for the electric machine is too small for the cycle. Torque will be scaled accordingly');

end

%% 4) Check traction limit and assign outputs
%From this point there are no more iterations through sim_acc, which is why the traction limit is checked here:
%Checking it inside acceleration_sim could cause an Errorlog every time the function is called.

%a) Check traction limit during acceleration from 0 to v_max:
if vehicle.LDS.sim_acc.exceeded_traction_limit

    vehicle.LDS = errorlog(vehicle.LDS,['The installed power causes a traction loss during the acceleration from 0 to ',num2str(vehicle.LDS.settings.v_max_sim),' km/h']);

end

%b) Check traction limit sim_cons:
[vehicle.LDS.sim_cons.traction_limit] = calc_traction_lim(vehicle.LDS, Par, vehicle.LDS.sim_cons.resistance.F_a,alpha); 

%check if acceleration is surpassed in any time step during the cycle
if ~(all (vehicle.LDS.sim_cons.traction_limit > a))

    vehicle.LDS = errorlog(vehicle.LDS,'The given acceleration in the driving cycle is above traction limit');

end

%% 5) Assign outputs

%P_batt vector without auxiliaries in kW
vehicle.LDS.sim_cons.P_batt  = vehicle.LDS.sim_cons.P_batt/1000;                                                 

%calculate P_batt with auxiliaries if activated in kW
if ~isnan(vehicle.LDS.settings.power_auxiliaries)
    
    vehicle.LDS.sim_cons.power_auxiliaries=vehicle.LDS.settings.power_auxiliaries;
    vehicle.LDS.sim_cons.P_batt = vehicle.LDS.sim_cons.P_batt + vehicle.LDS.settings.power_auxiliaries.*ones(length(vehicle.LDS.sim_cons.t),1);

else
    
    vehicle.LDS.sim_cons.power_auxiliaries=0;
    
end                                                                                         

%Assign battery Energy to sim_cons
vehicle.LDS.sim_cons.E_bat_step = vehicle.LDS.sim_cons.P_batt.*delta_t/3600;                                          %consumed energy per time step kW h
vehicle.LDS.sim_cons.E_bat_cum  = cumsum(vehicle.LDS.sim_cons.E_bat_step,'omitnan');                                  %cumulated energy vector in kW.h
vehicle.LDS.sim_cons.E_bat_sum  = vehicle.LDS.sim_cons.E_bat_cum(end);                                                %required total energy in kW h (scalar)
dist_vec                    = cumsum(v.*delta_t)/1000;                                                        %distance vector of drving cycle in km                                   
vehicle.LDS.sim_cons.dist       = dist_vec(length(v));                                                            %total distance of drving cycle in km
vehicle.LDS.sim_cons.consumption100km     = vehicle.LDS.sim_cons.E_bat_sum/vehicle.LDS.sim_cons.dist * 100;               %power consumption of 100km  
    
end

    
    
  