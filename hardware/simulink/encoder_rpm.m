function rpm = encoder_rpm(count)
%ENCODER_RPM Cumulative encoder count -> RPM using wall-clock dt.
%   tic/toc dt keeps the conversion correct when a tick stalls (e.g. Von
%   inference), which a fixed sample-time gain cannot. Emits a fresh
%   estimate every >=50 ms of wall time; holds the last value between.

persistent cprev tprev last

c = double(count);
if isempty(tprev)
    cprev = c; tprev = tic; last = 0;
    rpm = last;
    return
end

dt = toc(tprev);
rpm = last;
if dt >= 0.05
    rpm = -(60/3576) * (c - cprev) / dt;
    cprev = c; tprev = tic; last = rpm;
end
end
