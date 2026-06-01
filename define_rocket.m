function p = define_rocket()
% Rocket, motor and environment definition. Swap these for a real design.
% All quantities SI: metres, kilograms, seconds, newtons.
p.m_dry  = 1.20;
p.m_prop = 0.08;
p.d_body = 0.054;

p.Cd_ascent = 0.45;
p.Cd_chute  = 1.50;
p.d_chute   = 0.60;

p.t_burn = 1.3;
p.T_avg  = 115;
p.T_peak = 180;

p.t_deploy_delay = 1.0;   % chute opens this long after apogee

p.rho0    = 1.225;
p.g       = 9.81;
p.scale_h = 8500;
p.a_sound = 343;

% geometry used only by the stability check; x_cg from CAD mass props
p.L_nose   = 0.20;
p.L_body   = 0.75;
p.N_fins   = 4;
p.fin_root = 0.12;
p.fin_tip  = 0.05;
p.fin_span = 0.06;
p.fin_xpos = 0.87;
p.x_cg     = 0.55;
end
