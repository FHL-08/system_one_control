function plot_hw_results()
%PLOT_HW_RESULTS Export the README hardware figures from hw_*.mat.
%   Reads hw_von.mat / hw_pi.mat (written by run_hardware.m) and writes
%   hw_compare.png (overlaid Von vs PI speed + applied duty) and
%   hw_grades.png (Von membership grades) next to this file.

here = fileparts(mfilename('fullpath'));
von = load(fullfile(here, 'hw_von.mat'));
pir = load(fullfile(here, 'hw_pi.mat'));
ref = load_params().ref_rpm;

f = figure('Visible', 'off', 'Position', [0 0 1400 880]);
f.Theme = "dark";

subplot(2,1,1); hold on; grid on;
plot(von.rpm.Time, squeeze(von.rpm.Data), 'DisplayName', 'Von');
plot(pir.rpm.Time, squeeze(pir.rpm.Data), 'DisplayName', 'PI');
yline(ref, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
ylabel('Speed [RPM]'); ylim([0 50]);
title(sprintf('Von vs PI on hardware (%g RPM reference)', ref));
legend('Location', 'southeast');

subplot(2,1,2); hold on; grid on;
plot(von.duty.Time, squeeze(von.duty.Data), 'DisplayName', 'Von');
plot(pir.duty.Time, squeeze(pir.duty.Data), 'DisplayName', 'PI');
xlabel('Time [s]'); ylabel('PWM duty [0-255]'); ylim([0 255]);
title('Applied PWM duty');
legend('Location', 'southeast');

exportgraphics(f, fullfile(here, 'hw_compare.png'), 'Resolution', 150);

g = figure('Visible', 'off', 'Position', [0 0 1450 620]);
g.Theme = "dark";
hold on; grid on;
mu = squeeze(von.mu.Data);
if isvector(mu), mu = mu(:).'; end
stairs(von.mu.Time, mu.');
terms = {'far\_under','under','slightly\_under','near\_under','on\_target', ...
         'near\_over','slightly\_over','over','far\_over'};
legend(terms(1:size(mu,1)), 'Location', 'eastoutside');
xlabel('Time [s]'); ylabel('\mu_i'); ylim([0 1]);
title('Von membership grades');
exportgraphics(g, fullfile(here, 'hw_grades.png'), 'Resolution', 150);
end
