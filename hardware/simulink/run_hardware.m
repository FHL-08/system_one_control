function out = run_hardware(stopTime)
%RUN_HARDWARE Run motor_von_hw.slx in Connected IO mode on the Uno.
%   Model runs host-side in real time (pacing on); PWM/Encoder blocks
%   talk to the board over serial. CtrlSelect: in1 = Von, in2 = PI.

if nargin < 1, stopTime = 30; end

venv_py = fullfile(fileparts(mfilename('fullpath')), '..', '..', '.venv', 'bin', 'python');
pe = pyenv;
if pe.Status == "NotLoaded"
    pyenv('Version', venv_py, 'ExecutionMode', 'OutOfProcess');
elseif pe.Executable ~= venv_py
    error(['Python already loaded (%s). Restart MATLAB and rerun so the ' ...
           'venv interpreter can be selected.'], pe.Executable);
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

fprintf('Running Connected IO for %g s (first run uploads IO server)...\n', stopTime);
tic;
simOut = sim(mdl);
fprintf('Wall time: %.1f s\n', toc);
configset.internal.setParam(hCS, 'ConnectedIO', 'off', 'Apply', 'off');

out.rpm  = simOut.get('rpm_meas');
out.duty = simOut.get('duty_von');
out.mu   = simOut.get('mu_von');
out.u    = simOut.get('u_pid');
save('hardware_result.mat', '-struct', 'out');

ref = str2double(get_param([mdl '/Ref'], 'Value'));
f = figure('Name', 'Hardware run - motor_von_hw');
subplot(2,1,1); hold on; grid on;
plot(out.rpm.Time, squeeze(out.rpm.Data), 'b-');
yline(ref, 'k--', sprintf('%g RPM', ref));
ylabel('Speed [RPM]'); title('Measured motor speed');
subplot(2,1,2); hold on; grid on;
plot(out.duty.Time, squeeze(out.duty.Data)*255, 'r-');
xlabel('Time [s]'); ylabel('PWM duty [0-255]');
exportgraphics(f, 'hardware_run.png', 'Resolution', 150);
end
