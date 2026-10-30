> 24.10.2026
Both external simulators pass only what they're fitted to: bias and std. They
fail on the noise floor, cutoff, low_freq and most Allan points, because
neither models filters or drift. One fairness note: they beat timusim on gyro
bias (32/33 against 27/33). That's because each gets its own config's session-1
gyro mean, while timusim uses one zero level averaged over power-ups.

> 25.10.2026
Found that MATLAB's model is the richest of the four, supporting gain/bias, white/random-walk/quantization noise, and bias instability via an arbitrary IIR filter (default Kasdin 1/f approximation mapping to Allan floor = 0.664·B)

V2 scores 60 s means, so it mostly separates models that have a gain matrix from those that only have a bias.

V1 with gnss-ins-sim:
- Finger: it passes 21–36 of 60 metrics per config. IMUSim and WIMUSim pass 12–30, timusim 53–60.
- Palm: 22–38 for gnss-ins-sim, against 17–35 and 45–60.
- Its single drift term helps at long τ. It still has no filter or quantization, so it misses the same metrics the other two miss.

> 26.10.2026
The only failures are finger −x tilt, for WIMUSim and gnss-ins-sim. The real
tilt there is 3.04°, with a window of 1°. Those two models have no gain matrix,
so they simulate about 1.3° and land outside the window. The simulators with a
3x3 gain (timusim, IMUSim, MATLAB) get 2.35°. That's also the only place in V2
where the gain matrix makes the difference. Everywhere else, a zero-level
offset alone is enough.
