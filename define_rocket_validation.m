function p = define_rocket_validation()
% Validation case: matched to the OpenRocket build flown with an AeroTech
% H165R motor, for cross-checking simulate_flight against OpenRocket.
%
% OpenRocket reference results for this rocket + motor:
%   apogee 445 m, max velocity 103 m/s (Mach 0.30), max accel 126 m/s^2 (~12.8 g)
%
% Use with: f = simulate_flight(define_rocket_validation());

% mass: OpenRocket no-motor mass was 1222 g; motor adds ~202 g total.
% treat propellant as the burnable share, the rest as dry.
p.m_dry  = 1.31;     % rocket + motor casing (kg)
p.m_prop = 0.09;     % propellant burned (kg)
p.d_body = 0.054;

p.Cd_ascent = 0.45;
p.Cd_chute  = 1.50;
p.d_chute   = 0.60;

% AeroTech H165R (published): 161 N.s impulse, 165 N avg, ~230 N peak,
% ~0.96 s burn.
p.t_burn = 0.96;
p.T_avg  = 210;
p.T_peak = 280;

p.t_deploy_delay = 1.0;

p.rho0    = 1.225;
p.g       = 9.81;
p.scale_h = 8500;
p.a_sound = 343;

% geometry matched to the OpenRocket build
p.L_nose   = 0.20;
p.L_body   = 0.75;
p.N_fins   = 4;
p.fin_root = 0.12;
p.fin_tip  = 0.05;
p.fin_span = 0.06;
p.fin_xpos = 0.87;
p.x_cg     = 0.699;   % OpenRocket CG was 69.9 cm with motor
end