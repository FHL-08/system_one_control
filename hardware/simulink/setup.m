%SETUP One-time prep for motor_von_hw Connected IO runs.
%   Selects the project venv interpreter and warms the Von backend so the
%   first paced tick is not stalled by the model load.

venv_py = fullfile(fileparts(mfilename('fullpath')), '..', '..', '.venv', 'bin', 'python');
if ~strcmp(pyenv().Status, 'Loaded')
    pyenv('Version', venv_py, 'ExecutionMode', 'OutOfProcess')
end

load_system('motor_von_hw')
hCS = getActiveConfigSet('motor_von_hw');
codertarget.data.setIOBlocksMode(hCS, 'connected');
configset.internal.setParam(hCS, 'ConnectedIO', 'on', 'Apply', 'off');

params = load_params();
von_fuzzy(0, params.ref_rpm, 0, 0);  % loads the model
