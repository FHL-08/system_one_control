function [duty, mu] = von_fuzzy(rpm, target, duty_prev, t)
%VON_FUZZY MATLAB shim -> py.von_control.tick (the real controller).
%   t = sim time for the inference gate (cadence in controller_params.json).
%   All tuning constants live in controller_params.json via von_control.py.
%   rpm, target in RPM; duty_prev, duty in [0,1]; mu = 9x1 grades.

persistent ready

if isempty(ready)
    try
        py.importlib.import_module('von');   % triggers interpreter load
        insert(py.sys.path, int32(0), ...
            fullfile(fileparts(mfilename('fullpath')), '..', '..', 'shared'));
        py.importlib.import_module('von_control');
    catch ME
        error('von_fuzzy:NoPython', ...
            ['Cannot init von backend. Select the venv interpreter first:\n' ...
             '  pyenv(''Version'', <project>/.venv/bin/python, ' ...
             '''ExecutionMode'',''OutOfProcess'')\n%s'], ME.message);
    end
    ready = true;
end

out = py.von_control.tick(rpm, target, duty_prev, t);
duty = double(out{1});
mu = cellfun(@double, cell(out{2})).';
end
