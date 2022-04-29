function [ vehicle ] = errorlog( vehicle, msg )
%% Description:
% logs errors into vehicle struct with automatic determination of the 
% error producing function
% Author:   Steffen Hottner, FTM, TUM
% Date:     15.11.2019
%% Inputs:
%   vehicle:    struct with the vehicle parameters
%   msg:        error message to log
%% Outputs:
%   vehicle struct with ErrorLog
%% Implementation

% check if errorlog exists yet
if isfield(vehicle,'ErrorLog')
    % get number of existing entries
    k = numel(fieldnames(vehicle.ErrorLog));
else
    k = 0;
end
    % append as Error$k$
    Error = sprintf('Error%d', k+1);

    % get calling function from stack
    stack = dbstack(1);
    if numel(stack) < 1
        caller = 'can''t determine calling function for error';
    else
        caller = [stack(1).file ', line ' num2str(stack(1).line)];
    end
    
    % log message and calling function
    vehicle.ErrorLog.(Error)(1) = string(msg);
    vehicle.ErrorLog.(Error)(2) = string(['Error in function: ' caller]);
    
    %The user did not supress the warnings -> show them in the command window!
    if isfield(vehicle.settings,'suppress_LDS_warnings') %The structure which was passed to ErrorLog is the vehicle.LDS struct
        if vehicle.settings.suppress_LDS_warnings==0
            disp(msg)
        end
    else %The structure which was passed to the ErrorLog is the vehicle struct
        if vehicle.settings.suppress_warnings==0
            disp(msg)
        end
    end
end