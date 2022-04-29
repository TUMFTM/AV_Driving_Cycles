%% Description: preprocessing of driving cycle
% Designed by: Duan, Xucheng (FTM, Technical University of Munich)
%-------------
% Created on: 23.10.2021
% Last update: 05.11.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: preprocessing of driving cycle for AVDC-Simulation
%       Tasks: 1. interpolate driving cycle to 20 Hz
%              2. calculate positions
%              3. records type of road
%              4. records posibility of overtaking
%              5. calculate available distance for overtake
%
% ------------
% Input:    - tc: array of cycle time [s]
%           - vc: array of cycle speed [m/s]
%           - type: type of road ('urban','road','highway','auto')
%           - ovt: overtaking posibility ('always','never','auto')
%
% ------------
% Output:   - cycle: processed driving cycle data
%           
% ------------
% References:   
% 
%%-------------

function cycle = Preprocess_DC(tc, vc, type, ovt)
% 1. interpolate driving cycle to 20 Hz
    tc = tc - min(tc); % begin from 0
    cycle.t = 0:0.05:max(tc); % new time array at 20 Hz
    cycle.v = interp1(tc, vc, cycle.t, 'makima');
% 2. calculate positions
    cycle.x = cumtrapz(cycle.t, cycle.v ./ 3.6);
% 3. records type of road
    cycle.type = ones(size(cycle.t));
    switch type
        case 'urban'
            cycle.type = cycle.type .* 1;
        case 'road'
            cycle.type = cycle.type .* 2;
        case 'highway'
            cycle.type = cycle.type .* 3;
        case 'auto'
            % decide road type based on max segment speed
            % urban <= 60, 60 < road < 110, highway >= 110
            seg_begin = 1;
            stopped = 1;
            v_max_seg = 0;
            for ii = 1:length(cycle.t)
                if stopped
                    if cycle.v(ii) > 0
                        % end of segment
                        if v_max_seg <= 60
                            seg_type = 1; % urban
                        elseif v_max_seg >= 110
                            seg_type = 3; % highway
                        else
                            seg_type = 2; % road
                        end
                        for jj = seg_begin : ii - 1
                            cycle.type(jj) = seg_type;
                        end
                        seg_begin = ii; % begin of next segment
                        stopped = 0;
                        v_max_seg = cycle.v(ii);
                    end
                else
                    if cycle.v(ii) == 0
                        stopped = 1;
                    else
                        v_max_seg = max(v_max_seg, cycle.v(ii));
                    end
                end
            end
            % end of last segment
            if v_max_seg <= 60
                seg_type = 1; % urban
            elseif v_max_seg >= 110
                seg_type = 3; % highway
            else
                seg_type = 2; % road
            end
            for jj = seg_begin : ii - 1
                cycle.type(jj) = seg_type;
            end
            % error('Automatic analysis of road type is not programmed.')
        otherwise
            error('Invalid input for road type!')
    end
% 4. assess posibility of overtaking
    switch ovt
        case 'always'
            cycle.ovt = ones(size(cycle.t));
        case 'never'
            cycle.ovt = zeros(size(cycle.t));
        case 'auto'
            cycle.ovt = zeros(size(cycle.t));
            % Criteria: overtaking is allowed, when ...
            %    - speed within 15[km/h] for at least 20[s]
            %    - not slow: min speed > 30[km/h]
            v_window = 15;
            t_window = 20;
            v_min = 30;
            for ii = 1:length(cycle.t)
                i_seg = find(cycle.t >= cycle.t(ii) & ...
                    cycle.t < cycle.t(ii) + t_window);
                v_seg = cycle.v(i_seg);
                if min(v_seg) > v_min
                    if max(v_seg) - min(v_seg) <= v_window
                        cycle.ovt(ii) = 1;
                    end
                end
            end
        otherwise
            error('Invalid input for overtaking option!')
    end
% 5. calculate available distance for overtake
    cycle.dovt = zeros(size(cycle.t));
    for i_begin = 1:length(cycle.x)
        for ii = i_begin:length(cycle.x)
            if ~cycle.ovt(ii)
                break
            end
        end
        cycle.dovt(i_begin) = cycle.x(ii) - cycle.x(i_begin);
    end
% save settings
    cycle.settings.type = type;
    cycle.settings.overtake = ovt;
end