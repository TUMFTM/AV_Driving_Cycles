%% Description: function for calulating safety indicators
% Designed by: Duan, Xucheng (FTM, Technical University of Munich)
%-------------
% Created on: 14.09.2021
% Last update: 23.11.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: calculate safety indicators with speed profile
%              
% ------------
% Input:    - Logs: logged data from simulation
% ------------
% Output:   - RF: risk feeling according to [1]
%           - SM: safety margin according to [2]
% ------------
% References: [1] T. Kondoh, T. Yamamura, S. Kitazaki, N. Kuge, und E. R. Boer, 
%                 “Identification of Visual Cues and Quantification of Drivers' 
%                   Perception of Proximity Risk to the Lead Vehicle in Car-Following Situations,”
%                 JMTL, Bd. 1, Rn. 2, S. 170–180, 2008. DOI: 10.1299/jmtl.1.170
%             [2] X. Zhao et al, 
%                 “Risk perception and the warning strategy based on microscopic driving state,”
%                 Accident; analysis and prevention, Bd. 118, S. 154–165, 2018.
%                 DOI: 10.1016/j.aap.2018.02.012
%
%%-------------
function res = Eval_Safety(Logs)
    v_ego = Logs.v_ego.Data;
    v_front = Logs.v_front.Data;
    x_rel = Logs.x_rel.Data;
    res = struct();
    % [1]
    RF = 5 * (v_ego - v_front) ./ x_rel + ...
        v_front ./ x_rel;
    % [2]
%     SM = 1 - (0.15 * v_ego ./ x_rel + ...
%         (v_ego + v_front).*(v_ego - v_front)./(1.5 * 9.8 * x_rel));
    SMd = (0.15 * v_ego ./ x_rel + ...
         (v_ego + v_front).*(v_ego - v_front)./(1.5 * 9.8 * x_rel));
    % remove invalid points
    INVALID = x_rel<=0 | logical(Logs.state_overtake.Data);
    RF(INVALID) = [];
    RF(RF<0) = 0;
%     SM(INVALID) = [];
    SMd(INVALID) = [];
    SMd(SMd<0) = 0;
    % save results
    res.SM_rms = 1 - rms(SMd);
    res.SF = cdf('Normal',res.SM_rms,0.8617,0.1148) /.8858 * 6 + 4;
%     res.SF = corr_lim(res.SM_rms, 0.8, 1.0);
end