function pid = design_pid()
%DESIGN_PID Discrete 1-DOF PI design for the identified DC motor model.
%   Plant, reference and volts-per-duty come from shared/controller_params.json
%   (via load_params). Loop output is duty fraction [0,1] -> x255 -> PWM,
%   so the controller sees Gd(z) = volts_per_duty*G(z).
%
%   Block: 1-DOF PID, Forward Euler, UseKiTs off -> coefficient is I*Ts:
%       C(z) = Kp + I*Ts/(z-1)
%   char. poly: z^2 + (b Kp - (1+a)) z + (a - b Kp + b I Ts)
%       Kp = (1+a-p1-p2)/b,  I = (1-p1)(1-p2)/(b*Ts)
%
%   Candidates verified by simulating the real loop (P on error,
%   saturation, anti-windup clamping) at a ref step. The winning gains
%   are written back into shared/controller_params.json -> the PID block
%   in motor_von_hw.slx picks them up via the model InitFcn.

params = load_params();
Ts  = params.plant.ts;
a   = params.plant.a;
b   = params.volts_per_duty * params.plant.b;   % RPM per unit duty
ref = params.ref_rpm;
Gd = tf([0 b], [1 -a], Ts);

umax = 1.0;                 % duty fraction

best = [];
for wn = 1:0.5:12
    for zeta = [0.8 0.9 1.0 1.2 1.5 2.0]
        if zeta <= 1
            wd = wn*sqrt(1-zeta^2);
            s = -zeta*wn + 1j*wd*[1 -1];
        else
            s = -wn*(zeta + [-1 1]*sqrt(zeta^2-1));
        end
        p = exp(s*Ts);
        Kp = (1 + a - p(1) - p(2))/b;
        I  = (1 - p(1))*(1 - p(2))/(b*Ts);
        if Kp <= 0 || I <= 0
            continue
        end
        % simulate actual loop: P on error, forward-Euler I, clamp AW
        N = round(20/Ts);
        y = zeros(N,1); u = zeros(N,1); x = 0; integ = 0;
        for k = 2:N
            y(k) = x;
            e = ref - x;
            du = I*Ts*e;
            uUnsat = Kp*e + integ + du;
            uSat = min(umax, max(0, uUnsat));
            if ~(uUnsat > umax && du > 0) && ~(uUnsat < 0 && du < 0)
                integ = integ + du;          % frozen when pushing deeper
            end
            u(k) = uSat;
            x = a*x + b*uSat;                % one-step input delay
        end
        tt = (0:N-1)'*Ts;
        os = max(0, (max(y)-ref))/ref*100;
        rt = stepinfo(y, tt, ref).RiseTime;
        ess = ref - mean(y(end-100:end));
        if os < 5 && rt < 3
            score = abs(rt - 1.5) + 0.2*os + 0.5*abs(ess);
            if isempty(best) || score < best.score
                best = struct('Kp',Kp,'I',I,'wn',wn,'zeta',zeta, ...
                    'os',os,'rt',rt,'ess',ess,'score',score, ...
                    'upeak',max(u));
            end
        end
    end
end

assert(~isempty(best), 'No PI design met the specs.');

pid = best;
pid.Ts = Ts;
pid.Gd = Gd;
pid.Cz = tf([pid.Kp, -pid.Kp+pid.I*Ts], [1 -1], Ts);
pid.T_cl = feedback(pid.Cz*Gd, 1);

% write gains back to the shared params file (strip derived fields added
% by load_params — they are recomputed on every load)
params.pi.kp = pid.Kp;
params.pi.ki = pid.I;
params = rmfield(params, {'rpm_filter_num', 'rpm_filter_den'});
fid = fopen(fullfile(fileparts(mfilename('fullpath')), ...
                     '..', '..', 'shared', 'controller_params.json'), 'w');
fwrite(fid, jsonencode(params, 'PrettyPrint', true));
fclose(fid);

fprintf('PI: Kp = %.5f, I = %.5f (I*Ts = %.3e), Ts = %g\n', ...
    pid.Kp, pid.I, pid.I*Ts, Ts);
fprintf('Simulated: rise = %.2f s, overshoot = %.2f %%, ss err = %.2f RPM\n', ...
    pid.rt, pid.os, pid.ess);
fprintf('Gains written to controller_params.json\n');
end
