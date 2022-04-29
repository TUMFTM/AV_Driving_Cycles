%% Description: correlation with normal distribution
% Designed by: Duan, Xucheng (FTM, Technical University of Munich)
%-------------
% Created on: 22.11.2021
% Last update: 25.11.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: Correlation with normal distribution function.
%              Two set points are required for the correlation parameters.
%              (x1,y1)&(x2,y2) are usually a lower and a upper percentage,
%              therefore the limits for correlation could be defined.
%              The correlation (scaleing) has a S-shape according to the
%              normal distribution function.
%              With the corration curve defined with two set points, the
%              input value x is scaled to y.
%              
% ------------
% Input:    - x1,y1: value and percentage of 1st point
%           - x2,y2: value and percentage of 2nd point
%           - x: value to be scaled
% ------------
% Output:   - y: scaled value (0,1)
% ------------
% References: [1] SA Marc Raudszus, 
%                 Ableitung der funktionalen Anforderungen autonomer
%                 Fahrzeugkonzepte anhand ihrer Eigenschaften
%             [2] SA Leo Langer
%                 Ableitung konzeptbestimmender technischer Werte autonomer
%                 Fahrzeuge anhand einer Marktanalyse
%
%%-------------

function y = corr_lim(x, x1, x2, y1, y2)
%     k = log(0.05/29.75) / (x5 - x995);
%     x0 = (x5 - x995) * log(5) / log(0.05/29.75) + x5;
    k = (-log((10 - y1)/(y1 - 4)) + log((10 - y2)/(y2 - 4)))/(x1 - x2);
    x0 = (x2 * log((10 - y1)/(y1 - 4)) - x1 * log((10 - y2)/(y2 - 4))) / ...
        (log((10 - y1)/(y1 - 4)) - log((10 - y2)/(y2 - 4)));
    y = 6 ./ (1 + exp(-k .* (x - x0))) + 4;
end