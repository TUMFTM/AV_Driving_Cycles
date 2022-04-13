% RMS function
% MATLAB FUNCTION rms() requires Signal Processing Toolbox

function res = rms(x)
    res = sqrt(sum(x.*x)/length(x));
end