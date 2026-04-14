repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
origDir = pwd;
cleanupObj = onCleanup(@() cd(origDir)); %#ok<NASGU>
cd(repoRoot);

render_enable = false;
run(fullfile(repoRoot, 'Full_Sim_Init.m'));
load_system('Brown_6DOF_Plant_zach');

model = 'Brown_6DOF_Plant_zach';

speed = 70;
bank_deg = 0;

alpha0 = deg2rad(2.63250658095872);
theta0 = alpha0;
front0 = 1181.42797848053;
rear0 = 0;
de0 = deg2rad(-15.0602653566512);

pos_init = [0; 0; -1000];
omega_init = [0; 0; 0];
V_init = [speed * cos(alpha0); 0; speed * sin(alpha0)];
eul_init = [deg2rad(bank_deg); theta0; 0];

assignin('base', 'pos_init', pos_init);
assignin('base', 'omega_init', omega_init);
assignin('base', 'V_init', V_init);
assignin('base', 'eul_init', eul_init);

set_param(model, 'SimulationCommand', 'update');

q0 = localEulerToQuat(eul_init(1), eul_init(2), eul_init(3));
x0 = [pos_init; omega_init; V_init; q0];

u0 = zeros(24, 1);
u0(1:12) = 0;
u0(13:18) = 90;
u0(19) = front0;
u0(20) = rear0;
u0(21) = 0;
u0(22) = 0;
u0(23) = de0;
u0(24) = 0;

y0 = zeros(33, 1);
y0(10) = deg2rad(bank_deg);
y0(13:15) = 0;
y0(28) = speed;
y0(30) = 0;

iy = [10 13 14 15 28 30];
ix = 1:3;
iu = [1:18 21 22 24];
dx0 = zeros(13, 1);
idx = 4:9;

trimOptions = zeros(1, 18);
trimOptions(14) = 10000;

[xtrim, utrim, ytrim, dxtrim, opt] = trim(model, x0, u0, y0, ix, iu, iy, dx0, idx, trimOptions); %#ok<ASGLU>

trim_debug = struct();
trim_debug.x0 = x0;
trim_debug.u0 = u0;
trim_debug.xtrim = xtrim;
trim_debug.utrim = utrim;
trim_debug.ytrim = ytrim;
trim_debug.dxtrim = dxtrim;
trim_debug.max_dx = max(abs(dxtrim(idx)));
trim_debug.front_collective_rpm = utrim(19);
trim_debug.rear_collective_rpm = utrim(20);
trim_debug.delta_e_rad = utrim(23);
trim_debug.delta_e_deg = rad2deg(utrim(23));
trim_debug.roll_deg = rad2deg(ytrim(10));
trim_debug.pitch_deg = rad2deg(ytrim(11));
trim_debug.vinf_mps = ytrim(28);
trim_debug.alpha_deg = rad2deg(ytrim(29));
trim_debug.beta_deg = rad2deg(ytrim(30));

assignin('base', 'trim_debug', trim_debug);

fprintf('\nDirect trim result for %s\n', model);
fprintf('front collective rpm : %.6f\n', trim_debug.front_collective_rpm);
fprintf('rear collective rpm  : %.6f\n', trim_debug.rear_collective_rpm);
fprintf('delta_e deg          : %.6f\n', trim_debug.delta_e_deg);
fprintf('roll deg             : %.6f\n', trim_debug.roll_deg);
fprintf('pitch deg            : %.6f\n', trim_debug.pitch_deg);
fprintf('vinf m/s             : %.6f\n', trim_debug.vinf_mps);
fprintf('alpha deg            : %.6f\n', trim_debug.alpha_deg);
fprintf('beta deg             : %.6f\n', trim_debug.beta_deg);
fprintf('max |dx(4:9)|        : %.6e\n', trim_debug.max_dx);

if trim_debug.front_collective_rpm < 0 || abs(trim_debug.delta_e_deg) > 45
    warning('trim_brown6dofplant_zach_debug:NonphysicalResult', ...
        ['trim() converged numerically but landed on a nonphysical control input. ' ...
         'The direct plant trim is still underconstrained or missing actuator bounds.']);
end

close_system(model, 0);

function q = localEulerToQuat(phi, theta, psi)
cphi = cos(phi / 2); sphi = sin(phi / 2);
cth = cos(theta / 2); sth = sin(theta / 2);
cpsi = cos(psi / 2); spsi = sin(psi / 2);

q = [ ...
    cphi * cth * cpsi + sphi * sth * spsi; ...
    sphi * cth * cpsi - cphi * sth * spsi; ...
    cphi * sth * cpsi + sphi * cth * spsi; ...
    cphi * cth * spsi - sphi * sth * cpsi];
q = q / norm(q);
end
