function f = simulate_flight(p)
% Run the full ascent + recovery flight and return everything in a struct.

validate_rocket(p);

% derived quantities
p.A_body  = pi*(p.d_body/2)^2;
p.A_chute = pi*(p.d_chute/2)^2;
p.m_dot   = p.m_prop / p.t_burn;

% thrust curve: ignition spike, plateau, cubic drop to zero at burnout
p.t_thrust = linspace(0, p.t_burn, 100);
T = p.T_avg + (p.T_peak - p.T_avg).*exp(-5*p.t_thrust/p.t_burn) ...
            - p.T_avg.*(p.t_thrust/p.t_burn).^3;
p.T_curve = max(T, 0);

f.impulse = trapz(p.t_thrust, p.T_curve);
f.motor_class = classify_motor(f.impulse);
f.thrust_to_weight = p.T_avg / ((p.m_dry + p.m_prop)*p.g);

f.stability = compute_stability(p);

% two-phase integration: ascent to apogee, then recovery to ground.
% split because chute deployment is a discontinuity, and the event
% pins apogee exactly instead of guessing it from the samples.
y0 = [0; 0; p.m_dry + p.m_prop];
tol = odeset('RelTol',1e-7,'AbsTol',1e-9);

opts_up = odeset(tol, 'Events', @apogee_event);
[t1, y1] = ode45(@(t,y) equations_of_motion(t,y,p,false), [0 60], y0, opts_up);

t_apogee = t1(end);
opts_dn = odeset(tol, 'Events', @ground_event);
[t2, y2] = ode45(@(t,y) equations_of_motion(t,y,p,true), ...
                 [t_apogee 300], y1(end,:)', opts_dn);

t = [t1; t2(2:end)];
y = [y1; y2(2:end,:)];

f.t        = t;
f.altitude = y(:,1);
f.velocity = y(:,2);
f.mass     = y(:,3);
f.t_apogee = t_apogee;
f.params   = p;   % keep the resolved params with the result

% acceleration from the force balance (cleaner than differentiating v)
f.accel_g = zeros(size(t));
for i = 1:numel(t)
    chute = t(i) > (t_apogee + p.t_deploy_delay);
    dy = equations_of_motion(t(i), y(i,:)', p, chute);
    f.accel_g(i) = dy(2)/p.g;
end

% headline numbers
f.apogee   = max(f.altitude);
f.v_max    = max(f.velocity);
f.mach_max = f.v_max / p.a_sound;
f.a_max    = max(abs(f.accel_g));

rho = p.rho0 * exp(-max(f.altitude,0)/p.scale_h);
f.q_dyn = 0.5 * rho .* f.velocity.^2;
[f.max_q, f.i_maxq] = max(f.q_dyn);

settled = t > (t_apogee + p.t_deploy_delay + 0.5);
f.descent_rate = mean(abs(f.velocity(settled)));

[~, ib] = min(abs(t - p.t_burn));
f.v_burnout = f.velocity(ib);
end


function validate_rocket(p)
% Fail early and clearly on physically impossible inputs.
assert(p.m_dry  > 0, 'Dry mass must be positive.');
assert(p.m_prop > 0, 'Propellant mass must be positive.');
assert(p.t_burn > 0, 'Burn time must be positive.');
assert(p.T_avg  > 0, 'Average thrust must be positive.');
assert(p.d_body > 0 && p.d_chute > 0, 'Diameters must be positive.');
assert(p.T_avg > (p.m_dry+p.m_prop)*p.g, ...
       'Thrust below launch weight: rocket will not leave the pad.');
end


function dydt = equations_of_motion(t, y, p, chute_open)
% State y = [altitude; velocity; mass]. Returns its time derivative.
v = y(2);
m = y(3);

rho = p.rho0 * exp(-max(y(1),0)/p.scale_h);
T = interp1(p.t_thrust, p.T_curve, t, 'pchip', 0);

if chute_open
    Cd = p.Cd_chute; A = p.A_chute;
else
    Cd = p.Cd_ascent; A = p.A_body;
end

Fd = -0.5 * rho * Cd * A * v * abs(v);   % v*|v| keeps drag opposing motion

if t <= p.t_burn, dmdt = -p.m_dot; else, dmdt = 0; end

dydt = [v; (T + Fd)/m - p.g; dmdt];
end


function [val, isterm, dir] = apogee_event(~, y)
val = y(2); isterm = 1; dir = -1;   % velocity decreasing through zero
end

function [val, isterm, dir] = ground_event(~, y)
val = y(1); isterm = 1; dir = -1;   % altitude decreasing through zero
end


function cls = classify_motor(I)
edges = [2.5 5 10 20 40 80 160 320 640 1280];
names = {'A','B','C','D','E','F','G','H','I','J'};
idx = find(I <= edges, 1);
if isempty(idx), cls = 'K+'; else, cls = names{idx}; end
end


function s = compute_stability(p)
% Barrowman center-of-pressure estimate and static margin (calibers).
d = p.d_body;

CN_nose = 2.0;
X_nose  = 0.466 * p.L_nose;

Cr = p.fin_root; Ct = p.fin_tip; span = p.fin_span;
mid_chord = sqrt(span^2 + ((Cr-Ct)/2)^2);
Rb = d/2;
interference = 1 + Rb/(span + Rb);
CN_fins = interference * (4*p.N_fins*(span/d)^2) ...
          / (1 + sqrt(1 + (2*mid_chord/(Cr+Ct))^2));
X_fins = p.fin_xpos + (Cr-Ct)/3 * (Cr+2*Ct)/(Cr+Ct) ...
         + (1/6)*((Cr+Ct) - (Cr*Ct)/(Cr+Ct));

CN_total = CN_nose + CN_fins;
s.x_cp   = (CN_nose*X_nose + CN_fins*X_fins) / CN_total;
s.margin = (s.x_cp - p.x_cg)/d;
end
