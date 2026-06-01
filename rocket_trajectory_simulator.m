% Rocket flight simulator (1-DOF vertical).
% Defines a rocket, runs the flight model, prints results, plots them.
% All quantities SI: metres, kilograms, seconds, newtons.
%
% Uses shared functions: define_rocket.m, simulate_flight.m
% (keep all project files in the same folder).

clc; clear; close all;

rocket = define_rocket();
flight = simulate_flight(rocket);
report_flight(rocket, flight);
plot_flight(rocket, flight);


function report_flight(p, f)
fprintf('Motor      : %s class, %.0f N.s, burn %.2f s\n', ...
        f.motor_class, f.impulse, p.t_burn);
fprintf('Thrust/Wt  : %.1f : 1 at liftoff\n', f.thrust_to_weight);
fprintf('Stability  : CP %.3f m, CG %.3f m, margin %.2f cal\n', ...
        f.stability.x_cp, p.x_cg, f.stability.margin);
fprintf('Apogee     : %.0f m (%.0f ft) at t = %.2f s\n', ...
        f.apogee, f.apogee*3.281, f.t_apogee);
fprintf('Velocity   : burnout %.0f m/s, max %.0f m/s (Mach %.2f)\n', ...
        f.v_burnout, f.v_max, f.mach_max);
fprintf('Max accel  : %.1f g\n', f.a_max);
fprintf('Max-Q      : %.0f Pa at %.0f m\n', f.max_q, f.altitude(f.i_maxq));
fprintf('Descent    : %.1f m/s under chute, total flight %.0f s\n', ...
        f.descent_rate, f.t(end));

if f.stability.margin < 1
    fprintf('  note: static margin under 1 cal - marginal.\n');
elseif f.stability.margin > 2
    fprintf('  note: static margin over 2 cal - may weathercock.\n');
end
end


function plot_flight(p, f)
figure('Position',[50 50 1300 800]);

subplot(2,3,1);
plot(f.t, f.altitude, 'b','LineWidth',2); hold on;
xline(p.t_burn,'r--'); xline(f.t_apogee,'g--');
xlabel('Time (s)'); ylabel('Altitude (m)'); grid on;

subplot(2,3,2);
plot(f.t, f.velocity, 'r','LineWidth',2); hold on;
yline(0,'k--'); xline(f.t_apogee,'g--');
xlabel('Time (s)'); ylabel('Velocity (m/s)'); grid on;

subplot(2,3,3);
plot(f.t, f.accel_g, 'm','LineWidth',2); hold on;
yline(0,'k--'); xline(p.t_burn,'r--');
xlabel('Time (s)'); ylabel('Accel (g)'); grid on;

subplot(2,3,4);
plot(f.params.t_thrust, f.params.T_curve, 'Color',[0.9 0.4 0],'LineWidth',2);
xlabel('Time (s)'); ylabel('Thrust (N)'); grid on;
title(sprintf('%s | %.0f N.s', f.motor_class, f.impulse));

subplot(2,3,5);
plot(f.t, f.q_dyn,'Color',[0.5 0.2 0.7],'LineWidth',2);
xline(f.t(f.i_maxq),'k--');
xlabel('Time (s)'); ylabel('q (Pa)'); grid on;

subplot(2,3,6);
draw_stability(p, f.stability);
end


function draw_stability(p, s)
L = p.L_nose + p.L_body;
R = p.d_body/2;
hold on;
rectangle('Position',[p.L_nose, -R, p.L_body, 2*R],'Curvature',0.1, ...
          'FaceColor',[0.85 0.85 0.9]);
fill([0 p.L_nose p.L_nose],[0 R -R],[0.7 0.75 0.85]);
fill([p.fin_xpos p.fin_xpos+p.fin_root p.fin_xpos+p.fin_root], ...
     [-R -R -R-p.fin_span],[0.6 0.6 0.7]);
fill([p.fin_xpos p.fin_xpos+p.fin_root p.fin_xpos+p.fin_root], ...
     [R R R+p.fin_span],[0.6 0.6 0.7]);
plot(p.x_cg, 0,'bo','MarkerSize',12,'MarkerFaceColor','b');
plot(s.x_cp, 0,'r^','MarkerSize',12,'MarkerFaceColor','r');
text(p.x_cg, R*2.2,'CG','Color','b','HorizontalAlignment','center');
text(s.x_cp, -R*2.2,'CP','Color','r','HorizontalAlignment','center');
axis equal; xlim([-0.05 L+0.05]); ylim([-R*4 R*4]);
xlabel('Distance from nose (m)'); grid on;
end
