%% Description: function for getting Paraset
% Designed by: Duan, Xucheng (FTM, Technical University of Munich)
%-------------
% Created on: 07.11.2021
% Last update: 12.01.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: DoE factors -> AVDC_Sim Paraset
% 
% ------------
% Input:    - v: values of the 8 factors
%           
% ------------
% Output:   - Paraset: struct of parameters
%           
% ------------
% References:   - 
%%-------------
function Paraset = Get_Paraset(v)
    if size(v,2) ~= 8
        error('Get_Paraset: wrong number of factors (8)')
    end
    if size(v,1) ~= 1
        error('Get_Paraset: wrong number of factors (one row)')
    end
    Paraset = struct();
    Paraset.t_set = v(1);
    Paraset.P_a_y = [- v(2) * v(3), 0, v(2)] * 10;
    Paraset.P_v_y = [- v(4) * v(3), 0, v(4)] * 100;
    Paraset.set_speeds = v(5) * [50, 100, 130];
    % avoid overspeed on urban and road
    Paraset.set_speeds = min(Paraset.set_speeds,[50, 100, 300]);
    Paraset.a_max_y = v(6) * [2,2,1,1];
    Paraset.j_min_y = - v(7) * [1,1,.5,.5];
    Paraset.ovt_tol = v(8);
end