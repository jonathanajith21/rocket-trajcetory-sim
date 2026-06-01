% Motor selection study.
% Runs the same rocket on a range of motors, then picks the one that
% gets closest to a target apogee while keeping liftoff thrust-to-weight
% safe (>= 5:1). Reuses simulate_flight from the main simulator.
%
% Requires define_rocket.m and simulate_flight.m on the path.

clc; clear; close all;

target_apogee = 400;    % m - the altitude we're designing for
min_twr       = 5.0;    % minimum safe liftoff thrust-to-weight

% candidate motors: name, avg thrust (N), peak thrust (N), burn time (s)
motors = {
    'E-ish',  28,   45,  1.6
    'F-ish',  55,   80,  1.5
    'G-ish',  85,  130,  1.4
    'H-ish', 115,  180,  1.3
    'I-ish', 180,  280,  1.2
    'J-ish', 320,  480,  1.1
};

n = size(motors, 1);
results = struct('name',{},'apogee',{},'twr',{},'mach',{},'maxg',{},'safe',{});

for k = 1:n
    p = define_rocket();          % fresh baseline rocket each time
    p.T_avg  = motors{k,2};
    p.T_peak = motors{k,3};
    p.t_burn = motors{k,4};

    f = simulate_flight(p);

    results(k).name   = motors{k,1};
    results(k).apogee = f.apogee;
    results(k).twr    = f.thrust_to_weight;
    results(k).mach   = f.mach_max;
    results(k).maxg   = f.a_max;
    results(k).safe   = f.thrust_to_weight >= min_twr;
end

% pick best: smallest apogee error among the safe motors
err = inf(1,n);
for k = 1:n
    if results(k).safe
        err(k) = abs(results(k).apogee - target_apogee);
    end
end
[~, best] = min(err);

% --- table ---
fprintf('Target apogee: %d m   (min safe T/W: %.1f)\n\n', target_apogee, min_twr);
fprintf('%-8s %10s %8s %7s %7s %8s\n', ...
        'Motor','Apogee(m)','T/W','Mach','Max-g','Safe?');
for k = 1:n
    flag = '';
    if k == best, flag = '  <- best fit'; end
    fprintf('%-8s %10.0f %8.1f %7.2f %7.1f %8s%s\n', ...
        results(k).name, results(k).apogee, results(k).twr, ...
        results(k).mach, results(k).maxg, ...
        ternary(results(k).safe,'yes','NO'), flag);
end
fprintf('\nRecommended: %s (%.0f m, %.0f m from target)\n', ...
        results(best).name, results(best).apogee, ...
        abs(results(best).apogee - target_apogee));

% --- plots ---
apogees = [results.apogee];
twrs    = [results.twr];
safe    = [results.safe];

figure('Position',[80 80 1100 450]);

subplot(1,2,1);
b = bar(apogees,'FaceColor','flat');
for k = 1:n
    if ~safe(k),       b.CData(k,:) = [0.8 0.3 0.3];   % unsafe: red
    elseif k == best,  b.CData(k,:) = [0.2 0.7 0.3];   % best: green
    else,              b.CData(k,:) = [0.4 0.5 0.8];   % safe: blue
    end
end
hold on; yline(target_apogee,'k--','Target');
set(gca,'XTickLabel',{results.name});
ylabel('Apogee (m)'); title('Apogee by motor'); grid on;

subplot(1,2,2);
scatter(twrs, apogees, 80, 'filled'); hold on;
xline(min_twr,'r--','min safe T/W');
yline(target_apogee,'k--','target');
for k = 1:n
    text(twrs(k), apogees(k), ['  ' results(k).name],'FontSize',8);
end
xlabel('Liftoff thrust-to-weight'); ylabel('Apogee (m)');
title('Apogee vs thrust-to-weight'); grid on;


function out = ternary(cond, a, b)
if cond, out = a; else, out = b; end
end
