function out = run_hardware(stopTime)
%RUN_HARDWARE Run motor_von_hw.slx in Connected IO mode on the Uno.
%   Model runs host-side in real time (pacing on); PWM/Encoder blocks
%   talk to the board over serial. CtrlSelect is a ManualSwitch:
%   sw '1' = top input (Von), sw '0' = bottom input (PI).
%
%   Logs rpm_meas, duty_von (fraction), u_pid (counts), mu_von, and the
%   applied duty in counts. Saves hw_von.mat / hw_pi.mat + matching .png
%   next to this file, overwriting each run.

if nargin < 1, stopTime = 30; end
here = fileparts(mfilename('fullpath'));

venv_py = fullfile(here, '..', '..', '.venv', 'bin', 'python');
pe = pyenv;
if pe.Status == "NotLoaded"
    pyenv('Version', venv_py, 'ExecutionMode', 'OutOfProcess');
end

mdl = 'motor_von_hw';
load_system(mdl);
set_param(mdl, 'StopTime', num2str(stopTime));

% warm von_fuzzy so the ~5 s model load doesn't stall a paced tick
fprintf('Warming Von backend...\n');
params = load_params();
von_fuzzy(0, params.ref_rpm, 0, 0);

hCS = getActiveConfigSet(mdl);
codertarget.data.setIOBlocksMode(hCS, 'connected');
configset.internal.setParam(hCS, 'ConnectedIO', 'on', 'Apply', 'off');

sw0 = get_param([mdl '/CtrlSelect'], 'sw');

fprintf('Running Connected IO for %g s (first run uploads IO server)...\n', stopTime);
tic;
simOut = sim(mdl);
fprintf('Wall time: %.1f s\n', toc);
configset.internal.setParam(hCS, 'ConnectedIO', 'off', 'Apply', 'off');

sw = get_param([mdl '/CtrlSelect'], 'sw');
if ~strcmp(sw, sw0)
    warning('CtrlSelect changed during the run (%s -> %s); naming by final state.', sw0, sw);
end
ctrl = 'pi'; if strcmp(sw, '1'), ctrl = 'von'; end

out.ctrl     = ctrl;
out.rpm      = simOut.get('rpm_meas');
out.duty_von = simOut.get('duty_von');   % Von command, fraction [0-1]
out.u_pid    = simOut.get('u_pid');      % PI command, counts [0-255]
out.mu       = simOut.get('mu_von');     % Von membership grades

% duty that drove the motor, in PWM counts for both paths
out.duty = out.u_pid;
if strcmp(ctrl, 'von')
    out.duty = out.duty_von;
    out.duty.Data = out.duty.Data * 255;
end

base = fullfile(here, sprintf('hw_%s', ctrl));
save([base '.mat'], '-struct', 'out');
fprintf('Saved %s.mat\n', base);

ref = str2double(get_param([mdl '/Ref'], 'Value'));
f = figure('Name', sprintf('Hardware run - %s', ctrl));
subplot(2,1,1); hold on; grid on;
plot(out.rpm.Time, squeeze(out.rpm.Data), 'b-');
yline(ref, 'k--', sprintf('%g RPM', ref));
ylabel('Speed [RPM]'); title(sprintf('Measured motor speed (%s)', ctrl));
subplot(2,1,2); hold on; grid on;
plot(out.duty.Time, squeeze(out.duty.Data), 'r-');
xlabel('Time [s]'); ylabel('PWM duty [0-255]');
exportgraphics(f, [base '.png'], 'Resolution', 150);
end
