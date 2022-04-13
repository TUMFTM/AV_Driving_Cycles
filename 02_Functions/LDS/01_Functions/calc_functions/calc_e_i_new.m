function veh=calc_e_i_new(veh,ParLDS)
%% Description:
%If the e_i is a User-Input, then the Input will be assigned for further
%calculations, if not the Input will be calculated by using the reduced inertia from motor and inertia of wheels

%author:    Lorenzo Nicoletti, FTM, TUM 
%date:      27.05.2021
%update:    07.06.2021 - Korbinian Moller, TUM

%if more than one gear is available, e_i for all posssible gear
%combinations will be calculated
%for example:  # gear front:   2 gears - ratio [9.34, 7]
%              # gear rear:    3 gears - ratio [9.73, 5, 3]
%               
%              notation: gear_f,gear_r --> 1,2 means gear 1 at front axle, gear 2 at rear axle
%              e_i is a vector with e_i from gear combination [1,1 ; 1,2 ; 1,3 ; 2,1 ; 2,2 ; 2,3]
%% Inputs:
%masses and dimensions of tires, rims, gears
%% Outputs:
% The mass inertia factor reduced at the wheel center, e_i 
%% Sources: 
%% Implementation: 

%Determine number of gears
% gears front:
if ~isempty(veh.LDS.GEARBOX{1})
    num_gears_f = veh.LDS.GEARBOX{1}.num_gears;
else 
    num_gears_f = 1;
end
% gears rear:
if ~isempty(veh.LDS.GEARBOX{2})
    num_gears_r = veh.LDS.GEARBOX{2}.num_gears;
else
    num_gears_r = 1;
end

if ~isnan(veh.Input.e_i) %The User has assigned an e_i which should be used

    % repeat given e_i value - needed for multispeed transmission
    veh.LDS.parameters.e_i = ones(1,num_gears_f * num_gears_r) * veh.Input.e_i; 
    
else %The User did not assign an Input e_i
    
    %Here the are two options:
    %1) To exactly calculate the e_i, all the masses of the rotating
        %components need to be known. Therefore, the script must already have
        %finished an entire loop (including the mass calculation)
    
    %2) If this is the first loop of the script, the mass calculation has
        %not been completed yet. Therefore, the e_i cannot be calculated yet
        %and we have to use a typical value as starting point. 

    if isfield(veh.masses,'chassis') %Option 1) The mass of all rotating components is known

        %Reduced inertia front motors in kg m^2 
        if ~isempty(veh.LDS.MOTOR{1}) 
            J_red_f=veh.LDS.MOTOR{1}.J_M*veh.LDS.MOTOR{1}.quantity*veh.LDS.GEARBOX{1}.i_gearbox.^2;                     
        else
            J_red_f=0;  
        end

        %Reduced inertia rear motors in kg m^2
        if ~isempty(veh.LDS.MOTOR{2})
            J_red_r=veh.LDS.MOTOR{2}.J_M*veh.LDS.MOTOR{2}.quantity*veh.LDS.GEARBOX{2}.i_gearbox.^2;                      
        else
            J_red_r=0;
        end

        J_gearbox=zeros(2,1);
        
        %multispeed transmission - not implemented in gearbox model --> use typical value
        if num_gears_f > 1 || num_gears_r > 1
            J_gearbox = [0.168, 0.168];
        else
        % singlespeed transmission
        for axle=1:2
            if ~isempty(veh.gearbox{axle})
            gearbox=veh.gearbox{axle};

                if (strcmp(gearbox.Input.type, 'lay-shaft'))

                    gearbox = calc_J(gearbox);
                elseif (strcmp(gearbox.Input.type, 'planetary'))
                    % Calculation of moment of inertia in kg*mm^2
                    gearbox = calc_J_plan(gearbox);
                end
                
                J_gearbox(axle)=gearbox.results.J_tot_out*10^-6;
                
                %Check for errors (ONLY TEMPORARY UNTIL PETER CORECTS THE CODE)
                if J_gearbox(axle)<0 || J_gearbox(axle)>0.3
                    disp('Error in the calculation of the gearbox inertia. The values are not realistic and will be overwritten with typical values!');
                    J_gearbox(axle)=0.168;%Typical value for a gearbox (calculated from reference vehicle BMW i3)
                end
            end
        end
        end
        
        %mass of 4 wheels (calculated from the wheel modeling)
        m_wheels=veh.masses.chassis.tires_weight;               %All four tires in kg
        m_rims=veh.masses.chassis.rims_weight;                  %All four rims kg
        m_brakes=veh.masses.chassis.wheel_brakes_weight;        %All four brakes in kg
        r_wheel=veh.dimensions.CX.wheel_f_diameter/2000;        %in m
        r_rim=veh.dimensions.CX.rim_diameter_f_inch*25.4/2000;  %in m
        r_brake=veh.dimensions.CX.brake_disc_f_diameter/2000;   %in m

        %inertia of 4 tires ans 4 rims in kg m^2
        J_tires=0.5*m_wheels*(r_wheel^2+r_rim^2);  %Simplified as hollow cylinder with Ri=r_rim and Ro=R_tire
        J_rims=0.5*m_rims*(r_rim^2);               %Simplified as filled cylinder with R=Rrim
        J_brakes=0.5*m_brakes*(r_brake^2);         %Simplified as filled cylinder with R=rRbrake
        
        %addition of inertia wheels + front motor + rear motor to get total reduced inertia in kg m^2        
        J_red = zeros(1,num_gears_f * num_gears_r);
        k = 1;
        for i = 1:num_gears_f
            for j = 1:num_gears_r
                J_red(k) = J_red_f(i)+J_red_r(j) + J_tires + J_rims + J_brakes + sum(J_gearbox);  
                k = k + 1;
            end
        end
        
        %rotating mass factor [-]
        eps = J_red/(veh.LDS.weights.vehicle_empty_weight_EU*(veh.LDS.wheel.r_dyn/1000)^2);

        %e_i value [-]
        veh.LDS.parameters.e_i=eps+1;                                                                              

        %Assign Calculated Outputs:
        veh.LDS.wheel.J_tires=J_tires;
        veh.LDS.wheel.J_rims=J_rims;
        veh.LDS.wheel.J_brakes=J_brakes;
        veh.LDS.wheel.m_wheels=m_wheels;

    else %if the tire's width is not given, a fixed e_i value from the fixparameters is assigned for every gear   

        % repeat given e_i value from Parameters - needed for multispeed transmission
        veh.LDS.parameters.e_i = ones(1,num_gears_f * num_gears_r) * ParLDS.e_i; 
    end
end

end

