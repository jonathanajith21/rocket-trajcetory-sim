% Validation against OpenRocket.
% Runs the matched rocket + H165R motor through simulate_flight and
% compares apogee, max velocity and max acceleration to OpenRocket.
%
% Needs simulate_flight.m and define_rocket_validation.m on the path.

clc; clear; close all;

f = simulate_flight(define_rocket_validation());

% OpenRocket reference values for the same rocket + motor
or_apogee = 445;     % m
or_vmax   = 103;     % m/s
or_amax   = 126/9.81; % g  (OpenRocket reported 126 m/s^2)

sim_apogee = f.apogee;
sim_vmax   = f.v_max;
sim_amax   = f.a_max;

pct = @(sim,ref) 100*(sim-ref)/ref;

fprintf('Validation vs OpenRocket (AeroTech H165R)\n');
fprintf('%-18s %12s %12s %10s\n','Metric','This sim','OpenRocket','Diff %');
fprintf('%-18s %12.0f %12.0f %9.1f%%\n', 'Apogee (m)', ...
        sim_apogee, or_apogee, pct(sim_apogee,or_apogee));
fprintf('%-18s %12.0f %12.0f %9.1f%%\n', 'Max velocity (m/s)', ...
        sim_vmax, or_vmax, pct(sim_vmax,or_vmax));
fprintf('%-18s %12.1f %12.1f %9.1f%%\n', 'Max accel (g)', ...
        sim_amax, or_amax, pct(sim_amax,or_amax));

fprintf('\nSim total impulse: %.0f N.s  (real H165R: 161 N.s)\n', f.impulse);
