%OPTIMAL_MOTOR_IDENTIFICATION Fit a first-order IIR model to measured data.
%   Expects t, u, y in the workspace — logged by
%   generate_identification_data.slx or loaded from motor_data.mat.

clc;
close all;

%% Extract measured input/output data

% Extract data from Simulink structures
time = squeeze(t.signals.values);  % Time [s]
input = squeeze(u.signals.values); % Input Voltage [V] 
ouput = squeeze(y.signals.values); % Motor Speed [RPM]

% configure data into column vectors
time = time(:);
input = input(:);
output = ouput(:);

% Ensure all vectors have the same length
N = min([length(time), length(input), length(output)]);

time = time(1:N);
input = input(1:N);
output = output(1:N);

dt = time(2) - time(1); % sample time [s]

% FIR length
M = 4096;

A = toeplitz([input(1); zeros(M-1, 1)], input');
B = toeplitz(input, [input(1) zeros(1, M-1)]);
R_c = A*output; % cross correlation
R_a = A*B;      % auto correlation

% Regularized solve: ridge term damps the tap directions the step input
% barely excites (R_a is severely ill-conditioned, cond ~ 5.7e5)
lambda = 1e-5 * max(diag(R_a));
inv_R_a = (R_a + lambda*eye(M)) \ eye(M);

% Estimate FIR coefficients using the regularized inverse correlation matrix
firCoefficients = inv_R_a * R_c;

% use 0.5*MSE as objective function of unconstrained QP
E = 0.5*(firCoefficients')*R_a*firCoefficients...
    - (R_c')*firCoefficients + 0.5*(output'*output);
NE = (2*E)/(output'*output); % normalized residual energy
percentage_accuracy = 100*(1-NE);

% Frequency response of FIR filter
[H, omega] = freqz(firCoefficients, 1, N);

% Desired IIR filter order (1st order system)
iirOrder = 1; % number of poles

% The data contains a one-sample input-to-output delay that a strictly
% proper first-order model cannot represent, so remove it from H before
% fitting and put it back as a leading zero in the numerator.
iirNumeratorOrder = 0;
[ b, a ] = invfreqz(H.*exp(1j*omega), omega, iirNumeratorOrder, iirOrder, [], 500);
b = real(b); a = real(a);
a = a / a(1);
sys = tf([0 b], a, dt);

% Compare measured and modeled output responses
modeledOutput = lsim(sys, input, time);
figure;
plot(time, output, 'b', time, modeledOutput, 'r--', 'LineWidth', 1.2);
grid on;
legend('Measured output', 'IIR model output');
xlabel('Time [s]'); ylabel('Motor Speed [RPM]');