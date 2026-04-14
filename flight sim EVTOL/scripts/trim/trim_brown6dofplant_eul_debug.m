repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
origDir = pwd;
cleanupObj = onCleanup(@() cd(origDir)); %#ok<NASGU>
cd(repoRoot);

render_enable = false;
run(fullfile(repoRoot, 'Full_Sim_Init.m'));
use_avl_aero = false; %#ok<NASGU>
assignin('base', 'use_avl_aero', false);
load_system('Brown_6DOF_Plant_EUL');

model = 'Brown_6DOF_Plant_EUL';

speed = 70;
bank_deg = 0;

alpha0 = deg2rad(2.63250658095872);
theta0 = alpha0;
front0 = 1181.42797848053;
rear0 = 0;
de0 = deg2rad(-15.0602653566512);

pos_init = [0; 0; -1000];
V_init = [speed * cos(alpha0); 0; speed * sin(alpha0)];
eul_init = [deg2rad(bank_deg); theta0; 0];
omega_init = [0; 0; 0];

assignin('base', 'pos_init', pos_init);
assignin('base', 'V_init', V_init);
assignin('base', 'eul_init', eul_init);
assignin('base', 'omega_init', omega_init);

set_param(model, 'SimulationCommand', 'update');

% State order from direct simulation state logging:
% x = [eul(3); pos_NED(3); omega(3); V_B(3)]
x0 = [eul_init; pos_init; omega_init; V_init];

% Input order from top-level inports:
% 1:12 Motor_RPM_cmd, 13:18 Tilt_angles_cmd, 19 Front collective,
% 20 Rear collective, 21 delta_f, 22 delta_a, 23 delta_e, 24 delta_r
u0 = zeros(24, 1);
u0(1:12) = 0;
u0(13:18) = 90;
u0(19) = front0;
u0(20) = rear0;
u0(21) = 0;
u0(22) = 0;
u0(23) = de0;
u0(24) = 0;

% Output rows from linmod/output checks:
% 1:3 pos_NED, 4:6 V_B, 7:9 V_E, 10:12 eul, 13:15 omega,
% 16:24 C_NB, 25:27 V_BA, 28 vinf, 29 alpha, 30 beta, 31:33 specific force
y0 = zeros(30, 1);
y0(10) = deg2rad(bank_deg);
y0(13:15) = 0;
y0(28) = speed;
y0(30) = 0;

iy = [10 13 14 15 28 30];
ix = 4:6;
iu = [1:18 21 22 24];
dx0 = zeros(12, 1);
idx = 7:12;

trimOptions = zeros(1, 18);
trimOptions(14) = 10000;

[xtrim, utrim, ytrim, dxtrim] = trim(model, x0, u0, y0, ix, iu, iy, dx0, idx, trimOptions);

trim_debug_eul = struct();
trim_debug_eul.x0 = x0;
trim_debug_eul.u0 = u0;
trim_debug_eul.xtrim = xtrim;
trim_debug_eul.utrim = utrim;
trim_debug_eul.ytrim = ytrim;
trim_debug_eul.dxtrim = dxtrim;
trim_debug_eul.max_dx = max(abs(dxtrim(idx)));
trim_debug_eul.front_collective_rpm = utrim(19);
trim_debug_eul.rear_collective_rpm = utrim(20);
trim_debug_eul.delta_e_deg = rad2deg(utrim(23));
trim_debug_eul.roll_deg = rad2deg(ytrim(10));
trim_debug_eul.pitch_deg = rad2deg(ytrim(11));
trim_debug_eul.vinf_mps = ytrim(28);
trim_debug_eul.alpha_deg = rad2deg(ytrim(29));
trim_debug_eul.beta_deg = rad2deg(ytrim(30));
trim_debug_eul.omega = ytrim(13:15);
trim_debug_eul.output_residual = ytrim(iy) - y0(iy);

assignin('base', 'trim_debug_eul', trim_debug_eul);

fprintf('\nDirect trim result for %s\n', model);
fprintf('front collective rpm : %.6f\n', trim_debug_eul.front_collective_rpm);
fprintf('rear collective rpm  : %.6f\n', trim_debug_eul.rear_collective_rpm);
fprintf('delta_e deg          : %.6f\n', trim_debug_eul.delta_e_deg);
fprintf('roll deg             : %.6f\n', trim_debug_eul.roll_deg);
fprintf('pitch deg            : %.6f\n', trim_debug_eul.pitch_deg);
fprintf('vinf m/s             : %.6f\n', trim_debug_eul.vinf_mps);
fprintf('alpha deg            : %.6f\n', trim_debug_eul.alpha_deg);
fprintf('beta deg             : %.6f\n', trim_debug_eul.beta_deg);
fprintf('omega [p q r]        : [%.6e %.6e %.6e]\n', trim_debug_eul.omega(1), trim_debug_eul.omega(2), trim_debug_eul.omega(3));
fprintf('max |dx(idx)|        : %.6e\n', trim_debug_eul.max_dx);
fprintf('max |y(iy)-y0(iy)|   : %.6e\n', max(abs(trim_debug_eul.output_residual)));

close_system(model, 0);
