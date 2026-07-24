function results = identify_gimbal_dynamics(csvFile, opts)
%IDENTIFY_GIMBAL_DYNAMICS  Identify gimbal YAW-axis physical parameters.
%
%   results = IDENTIFY_GIMBAL_DYNAMICS(csvFile) reads a VOFA+ exported CSV
%   log and fits the yaw-axis rigid-body friction model
%
%       tau = J*acc + B*vel + Fc*sign(vel) + Tau0
%
%   with (bounded) linear least squares. J is rotational inertia, B viscous
%   damping, Fc Coulomb friction magnitude and Tau0 a constant torque bias.
%
%   The time base comes ONLY from the VOFA timestamp in column I3 — there is
%   no fixed-Ts fallback. Acceleration is always derived by differentiating
%   the measured velocity (I2) against that real timestamp.
%
%   CSV column mapping (VOFA+ channels I0..I15, 1-based columns):
%       col 1 (I0) = yaw torque
%       col 2 (I1) = yaw position   [rad]
%       col 3 (I2) = yaw velocity   [rad/s]
%       col 4 (I3) = monotonic timestamp [ms]
%
%   Name-Value options:
%       AxisName           "yaw" (only yaw is supported)
%       SmoothMethod       "none"(default)|"sgolay"|"movmean"|"gaussian"
%       SmoothWindow       smoothing window length (default 15)
%       VelocityDeadband   |vel| below this is discarded (default 0.02)
%       AccelerationSource "diff_velocity"(default)|"sgolay_velocity"
%       MakePlots          show fit/residual plots (default true)
%       UseConstrainedFit  force J,B,Fc >= 0 (default true)
%       MaxPositionStep    discard position jumps above this (default Inf)
%       OutlierPadding     samples removed around each jump (default 0)
%       TorqueDelaySamples integer delay(s) to search, e.g. -6:6 (default 0)
%
%   results.Yaw.Parameters   .J .B .Fc .Tau0
%   results.Yaw.RMSE / .RSquared
%   results.Yaw.Diagnostics  .RemovedOutlierSamples .SelectedTorqueDelaySamples
%                            .NumSamplesUsed .AverageSampleTime
%   results.SampleTime       effective sample time [s]
%   results.CMacroText       ready-to-paste C #define block
%
%   For noisy real logs, "sgolay_velocity" acceleration usually gives the
%   most trustworthy inertia estimate.

arguments
    csvFile (1,1) string {mustBeNonzeroLengthText}
    opts.AxisName (1,1) string {mustBeMember(opts.AxisName, "yaw")} = "yaw"
    opts.SmoothMethod (1,1) string ...
        {mustBeMember(opts.SmoothMethod, ["none","sgolay","movmean","gaussian"])} = "none"
    opts.SmoothWindow (1,1) double {mustBePositive} = 15
    opts.VelocityDeadband (1,1) double {mustBeNonnegative} = 0.02
    opts.AccelerationSource (1,1) string ...
        {mustBeMember(opts.AccelerationSource, ["diff_velocity","sgolay_velocity"])} = "diff_velocity"
    opts.MakePlots (1,1) logical = true
    opts.UseConstrainedFit (1,1) logical = true
    opts.MaxPositionStep (1,1) double {mustBePositive} = Inf
    opts.OutlierPadding (1,1) double {mustBeNonnegative, mustBeInteger} = 0
    opts.TorqueDelaySamples (1,:) double {mustBeInteger} = 0
end

% --- YAW column mapping (1-based) ---------------------------------------
COL_TORQUE = 1;   % I0
COL_POS    = 2;   % I1
COL_VEL    = 3;   % I2
COL_TIME   = 4;   % I3 (timestamp, ms)

raw = table2array(readtable(csvFile));
n = size(raw, 1);
torqueRaw   = raw(:, COL_TORQUE);
positionRaw = raw(:, COL_POS);
velocityRaw = raw(:, COL_VEL);

% --- Time base (VOFA timestamp only) ------------------------------------
tMs = raw(:, COL_TIME);
t = (tMs - tMs(1)) / 1000;              % ms -> s, zero-based
dt = diff(t);
dtPos = dt(dt > 0);
if numel(dtPos) < 2
    error("identify_gimbal_dynamics:badTimestamp", ...
        "Column I3 does not contain a usable monotonic ms timestamp.");
end
avgTs = mean(dtPos);

% --- Acceleration (always differentiated from velocity) -----------------
switch opts.AccelerationSource
    case "diff_velocity"
        accelerationRaw = gradient(velocityRaw, t);
    case "sgolay_velocity"
        velForAcc = smoothdata(velocityRaw, "sgolay", opts.SmoothWindow);
        accelerationRaw = gradient(velForAcc, t);
end

% --- Optional smoothing -------------------------------------------------
torque = torqueRaw;
velocity = velocityRaw;
acceleration = accelerationRaw;
if opts.SmoothMethod ~= "none"
    torque       = smoothdata(torque, opts.SmoothMethod, opts.SmoothWindow);
    velocity     = smoothdata(velocity, opts.SmoothMethod, opts.SmoothWindow);
    acceleration = smoothdata(acceleration, opts.SmoothMethod, opts.SmoothWindow);
end

% --- Position-jump outlier mask -----------------------------------------
outlierMask = false(n, 1);
if isfinite(opts.MaxPositionStep)
    posStep = [0; abs(diff(positionRaw))];
    jumpMask = posStep > opts.MaxPositionStep;
    outlierMask = dilateMask(jumpMask, opts.OutlierPadding);
end
removedOutlierSamples = nnz(outlierMask);

% --- Torque-delay search ------------------------------------------------
bestRMSE = Inf;
best = struct("theta", [0;0;0;0], "delay", 0, "used", 0);
for d = opts.TorqueDelaySamples
    [tauA, accA, velA, outA] = alignByDelay(torque, acceleration, velocity, outlierMask, d);
    keep = ~outA & abs(velA) >= opts.VelocityDeadband;
    if nnz(keep) < 4
        continue
    end
    Phi = [accA(keep), velA(keep), sign(velA(keep)), ones(nnz(keep), 1)];
    y = tauA(keep);
    theta = solveFit(Phi, y, opts.UseConstrainedFit);
    residual = y - Phi * theta;
    rmse = sqrt(mean(residual.^2));
    if rmse < bestRMSE
        bestRMSE = rmse;
        best.theta = theta;
        best.delay = d;
        best.used = nnz(keep);
        best.Phi = Phi;
        best.y = y;
    end
end

theta = best.theta;
J = theta(1); B = theta(2); Fc = theta(3); Tau0 = theta(4);
yPred = best.Phi * theta;
resid = best.y - yPred;
sse = sum(resid.^2);
sst = sum((best.y - mean(best.y)).^2);
rSquared = 1 - sse / sst;

% --- Assemble results ---------------------------------------------------
results = struct();
results.Yaw.Parameters = struct("J", J, "B", B, "Fc", Fc, "Tau0", Tau0);
results.Yaw.RMSE = bestRMSE;
results.Yaw.RSquared = rSquared;
results.Yaw.Diagnostics = struct( ...
    "RemovedOutlierSamples", removedOutlierSamples, ...
    "SelectedTorqueDelaySamples", best.delay, ...
    "NumSamplesUsed", best.used, ...
    "AverageSampleTime", avgTs);
results.SampleTime = avgTs;
results.CMacroText = buildMacroText(J, B, Fc, Tau0);

if opts.MakePlots
    plotFit(best.y, yPred, resid);
end
end

% ========================================================================
function [tauA, accA, velA, outA] = alignByDelay(torque, acc, vel, out, d)
% Positive d: torque lags the state by d samples (measured torque arrives
% late), so shift torque back by d to realign with state variables.
n = numel(torque);
if d >= 0
    idxState  = 1:n-d;
    idxTorque = 1+d:n;
else
    idxState  = 1-d:n;
    idxTorque = 1:n+d;
end
tauA = torque(idxTorque);
accA = acc(idxState);
velA = vel(idxState);
outA = out(idxState);
end

% ========================================================================
function theta = solveFit(Phi, y, useConstrained)
if ~useConstrained
    theta = Phi \ y;
    return
end
% Bounded LS: J,B,Fc >= 0, Tau0 free. Exact active-set enumeration
% (no Optimization Toolbox dependency) over which of the 3 bounded
% coefficients are pinned to zero; pick the feasible min-SSE solution.
bestSSE = Inf;
theta = zeros(4, 1);
for mask = 0:7
    pinned = bitget(mask, 1:3) == 1;   % which of [J B Fc] pinned to 0
    free = [~pinned, true];            % Tau0 always free
    cand = zeros(4, 1);
    cand(free) = Phi(:, free) \ y;
    if any(cand(1:3) < -1e-12)
        continue
    end
    sse = sum((y - Phi * cand).^2);
    if sse < bestSSE
        bestSSE = sse;
        theta = cand;
    end
end
end

% ========================================================================
function m = dilateMask(mask, pad)
m = mask;
if pad <= 0
    return
end
idx = find(mask);
for k = idx.'
    lo = max(1, k - pad);
    hi = min(numel(mask), k + pad);
    m(lo:hi) = true;
end
end

% ========================================================================
function txt = buildMacroText(J, B, Fc, Tau0)
lines = [
    "// Gimbal YAW dynamics: tau = J*acc + B*vel + Fc*sign(vel) + Tau0"
    sprintf("#define GIMBAL_YAW_J     %.6ff", J)
    sprintf("#define GIMBAL_YAW_B     %.6ff", B)
    sprintf("#define GIMBAL_YAW_FC    %.6ff", Fc)
    sprintf("#define GIMBAL_YAW_TAU0  %.6ff", Tau0)
    ];
txt = strjoin(lines, newline);
end

% ========================================================================
function plotFit(yMeas, yPred, resid)
figure("Name", "Gimbal YAW identification");
subplot(2, 1, 1);
plot(yMeas, "b"); hold on; plot(yPred, "r--"); hold off;
legend("measured", "predicted"); ylabel("torque"); grid on;
title("YAW torque fit");
subplot(2, 1, 2);
plot(resid, "k"); ylabel("residual"); xlabel("sample"); grid on;
title(sprintf("residual (RMSE = %.4g)", sqrt(mean(resid.^2))));
end



