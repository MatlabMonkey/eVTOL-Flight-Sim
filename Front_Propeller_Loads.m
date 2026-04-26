function [F_cg, M_cg] = Front_Propeller_Loads(RPM, RPM_dot, tilt_deg, tilt_dot_deg_s, omega, pivot_pos, hub_offset, spin_dir, kT, kQ, Jr, CG)

%#codegen
% FRONT PROP GROUP LOADS
% Inputs
%   RPM, RPM_dot        : scalar group RPM and RPM/s
%   tilt_deg            : scalar front-group tilt angle [deg]
%   tilt_dot_deg_s      : scalar front-group tilt rate [deg/s]
%   omega               : 3x1 body rates [p;q;r] in rad/s
%   CG                  : 3x1 center of gravity in body axes
% Parameters
%   pivot_pos           : 3x3, one rotor pivot per row [x y z]
%   hub_offset          : scalar hub standoff from pivot [m]
%   spin_dir            : 3x1, +1/-1 for each rotor
%   kT, kQ              : thrust / reaction-torque coefficients (RPM based)
%   Jr                  : rotor polar inertia for ONE rotor [kg*m^2]

omega = omega(:);
CG = CG(:);
spin_dir = spin_dir(:);

F_cg = zeros(3,1);
M_cg = zeros(3,1);

tilt_rad = (pi/180) * tilt_deg;
tilt_dot_rad_s = (pi/180) * tilt_dot_deg_s;

% Shared shaft direction for all three front rotors in this group
n = [sin(tilt_rad); 0; -cos(tilt_rad)];
n_dot = tilt_dot_rad_s * [cos(tilt_rad); 0; sin(tilt_rad)];

% Shared thrust / speed terms for grouped RPM
T_mag = kT * RPM * RPM;
Q_base = kQ * RPM * RPM;
Omega = (2*pi/60) * RPM;
Omega_dot = (2*pi/60) * RPM_dot;

F_motor = T_mag * n;

for i = 1:3
    s_i = spin_dir(i);
    pivot_i = pivot_pos(i,:).';

    % Rotor hub location and moment arm
    r_hub = pivot_i + hub_offset * n;
    r_arm = r_hub - CG;

    % Existing terms
    M_arm = cross(r_arm, F_motor);
    M_reaction = -(Q_base * s_i) * n;

    % Rotor angular momentum term
    % Includes:
    %   1) spin-up / spin-down torque: Jr * Omega_dot
    %   2) shaft reorientation from tilt: Jr * Omega * n_dot
    %   3) gyroscopic coupling: omega x h
    h_i = Jr * s_i * Omega * n;
    M_momentum = -(Jr * s_i * (Omega_dot * n + Omega * n_dot) + cross(omega, h_i));

    F_cg = F_cg + F_motor;
    M_cg = M_cg + M_arm + M_reaction + M_momentum;
end
end
