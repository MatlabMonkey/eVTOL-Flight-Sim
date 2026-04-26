function [] = root_locus_plot(sys_ss_13state)
% Root locus inspection
% rlocus only applies to SISO loops, so build a few representative
% cruise channels from the 13-state plant.
%
% sys_ss_13state state order:
% [phi theta psi u v w P Q R dLW dRW dLT dRT]
%
% sys_ss_13state input order:
% [front_coll_rpm deltaLW deltaRW deltaLT deltaRT]
%
% Build virtual control channels:
%   front_coll  = front collective
%   wing_diff   = differential wing surfaces
%   wing_sym    = symmetric wing surfaces
%   tail_sym    = symmetric tail surfaces
%   tail_diff   = differential tail surfaces

mix_virtual = [ ...
    1.0,   0.0,   0.0,   0.0,   0.0; ...
    0.0,  -0.5,   0.5,   0.0,   0.0; ...
    0.0,   0.5,   0.5,   0.0,   0.0; ...
    0.0,   0.0,   0.0,   0.5,  -0.5; ...
    0.0,   0.0,   0.0,   0.5,   0.5  ...
];

sys_ss_13_virtual = ss( ...
    sys_ss_13state.A, ...
    sys_ss_13state.B * mix_virtual, ...
    sys_ss_13state.C, ...
    sys_ss_13state.D * mix_virtual);

sys_ss_13_virtual.StateName = sys_ss_13state.StateName;
sys_ss_13_virtual.InputName = { ...
    'front_coll_rpm', ...
    'wing_diff', ...
    'wing_sym', ...
    'tail_sym', ...
    'tail_diff'};

% State indices in the 13-state reduced model
i_phi   = 1;
i_theta = 2;
i_psi   = 3;
i_u     = 4;
i_v     = 5;
i_w     = 6;
i_P     = 7;
i_Q     = 8;
i_R     = 9;

% Virtual input indices
j_front     = 1;
j_wing_diff = 2;
j_wing_sym  = 3;
j_tail_sym  = 4;
j_tail_diff = 5;

figure('Name', 'EVTOL cruise root locus channels');

subplot(2,3,1);
rlocus(sys_ss_13_virtual(i_u, j_front));
grid on;
title('u / front collective');

subplot(2,3,2);
rlocus(sys_ss_13_virtual(i_phi, j_wing_diff));
grid on;
title('phi / wing differential');

subplot(2,3,3);
rlocus(sys_ss_13_virtual(i_theta, j_tail_sym));
grid on;
title('theta / tail symmetric');

subplot(2,3,4);
rlocus(sys_ss_13_virtual(i_P, j_wing_diff));
grid on;
title('P / wing differential');

subplot(2,3,5);
rlocus(sys_ss_13_virtual(i_Q, j_tail_sym));
grid on;
title('Q / tail symmetric');

subplot(2,3,6);
rlocus(sys_ss_13_virtual(i_R, j_tail_diff));
grid on;
title('R / tail differential');
end