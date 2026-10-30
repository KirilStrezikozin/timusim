% Copyright (C) 2026 Kiril V. Strezikozin
% SPDX-License-Identifier: Apache-2.0
function timusim_hold(jobPath, outPath)
% A still hold through imuSensor: gain matrix and zero level, white noise, bias instability
% (fractalcoef 1/f filter), rate random walk, then saturation and rounding to the LSB.
job = jsondecode(fileread(jobPath));
fs = job.odr_hz;
n = round(job.seconds * fs);
% imuSensor's own gravity, so that 1 g reads as 1 g.
g = 9.81;
up = job.up(:)' / norm(job.up);

% The frame rotation that takes the ideal reading at identity orientation onto up.
r0 = idealReading(fs, eye(3));
R = alignment(r0 / norm(r0), up);
if norm(idealReading(fs, R) / norm(r0) - up) > 1e-9
    R = R';
end

imu = imuSensor("accel-gyro", SampleRate=fs, RandomStream="mt19937ar with seed", Seed=job.seed);
imu.Accelerometer = params(@accelparams, job.accel, g, fs, job.seconds);
imu.Gyroscope = params(@gyroparams, job.gyro, pi / 180, fs, job.seconds);
[accel, gyro] = imu(zeros(n, 3), zeros(n, 3), repmat(R, 1, 1, n));
accel_g = accel / g;
gyro_dps = rad2deg(gyro);
odr_hz = fs;
save(outPath, "accel_g", "gyro_dps", "odr_hz", "-v7");
end

function p = params(make, spec, si, fs, seconds)
% Densities are double-sided, as imuSensor's default NoiseType: the per-sample white std is
% NoiseDensity * sqrt(fs/2) and random-walk increments have std RandomWalk / sqrt(fs/2).
lsb = spec.lsb * si;
p = make(MeasurementRange=2^15 * lsb, Resolution=lsb, ConstantBias=spec.bias(:)' * si, ...
    NoiseDensity=spec.sd(:)' * si * sqrt(2 / fs));
if isfield(spec, "gain")
    p.AxesMisalignment = 100 * spec.gain;
end
if isfield(spec, "flicker")
    % Allan floor of the 1/f term is sqrt(2 ln 2 / pi) * BiasInstability; the filter needs
    % about fs * tau poles to stay 1/f out to tau, the longest tau the Allan curves reach.
    p.BiasInstability = spec.flicker.allan_floor(:)' * si / sqrt(2 * log(2) / pi);
    p.BiasInstabilityCoefficients = fractalcoef(ceil(fs * seconds / 5));
    p.RandomWalk = spec.flicker.rate_random_walk(:)' * si / sqrt(2);
end
end

function r = idealReading(fs, R)
imu = imuSensor("accel-gyro", SampleRate=fs);
r = imu(zeros(1, 3), zeros(1, 3), R);
end

function R = alignment(a, b)
% Rotation matrix taking unit vector a to unit vector b, half a turn when they are opposite.
v = cross(a, b);
c = dot(a, b);
if norm(v) < 1e-12
    if c > 0
        R = eye(3);
        return
    end
    [~, i] = min(abs(a));
    e = zeros(1, 3);
    e(i) = 1;
    v = cross(a, e);
    v = v / norm(v);
    R = 2 * (v' * v) - eye(3);
    return
end
K = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
R = eye(3) + K + K * K / (1 + c);
end
