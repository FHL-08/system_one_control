function params = load_params()
%LOAD_PARAMS Single source of truth for controller constants.
%   Reads shared/controller_params.json. Called from
%   motor_von_hw's InitFcn so block params (PID P/I, Ref) follow the JSON.
params = jsondecode(fileread(fullfile(fileparts(mfilename('fullpath')), ...
    '..', '..', 'shared', 'controller_params.json')));

% RPMFilter discrete coefficients at the model tick (first-order LPF,
% DC gain 1): num = 1-p, den = [1 -p], p = exp(-ts/tau)
p = exp(-params.plant.ts / params.rpm_filter_tau);
params.rpm_filter_num = 1 - p;
params.rpm_filter_den = [1, -p];
end
