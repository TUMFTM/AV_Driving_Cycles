%% Description: function for evaluating travel speed
% Designed by: Duan, Xucheng (FTM, Technical University of Munich)
%-------------
% Created on: 21.09.2021
% Last update: 05.11.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: evaluate travel speed
%              Speed Factor = Consumed Time / Minimum Time
% ------------
% Input:    - t: array of time [s]
%           - v: array of speed [m/s]
% ------------
% Output:   - res: struct of comfort indicators
%           
% ------------
% References:   - 
%%-------------
function res = Eval_Speed(Logs,Cycle)
    res = struct();
    % Consumed Time
    t_cons = max(Logs.v_ego.Time);
    % min Time
    cycle_xx = Cycle.x;
    cycle_type = Cycle.type;
    cycle_dovt = Cycle.dovt;
    ii = 2;
    while ii <= length(cycle_xx)
        if cycle_xx(ii) <= cycle_xx(ii-1)
            cycle_type(ii-1) = min(cycle_type(ii-1), cycle_type(ii));
            cycle_dovt(ii-1) = min(cycle_dovt(ii-1), cycle_dovt(ii));
            cycle_xx(ii) = [];
            cycle_type(ii) = [];
            cycle_dovt(ii) = [];
        else
            ii = ii + 1;
        end
    end
    max_speed = zeros(size(cycle_xx));
    for ii = 1:length(cycle_xx)
        switch cycle_type(ii)
            case 1
                max_speed(ii) = 50;
            case 2
                max_speed(ii) = 100;
            case 3
                max_speed(ii) = 130;
            otherwise
                error('Eval_Speed: invalid road type')
        end
    end
    max_speed = max_speed ./ 3.6; % km/h -> m/s
    dx = diff(cycle_xx);
    max_speed(1) = [];
    t_min = sum(dx ./ max_speed);
    % evaluation
    res = t_cons / t_min;
end