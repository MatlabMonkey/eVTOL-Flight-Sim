%% =========================================================================
%% 1. INITIALIZATION & AIRCRAFT PARAMETERS
%% =========================================================================
clear; close all; clc;

% Flight state (1 = cruise, 0 = hover)
flight_mode = 0;  
tilt_angle = 90 * flight_mode; 

% Physical Environment
rho = 1.225;
g   = -9.81; % Gravity defined for kinematics block

% --- Propeller Struct Constants ---
prop.hub_offset = 0.50;  % Standoff length from the pivot axis to the prop hub!
prop.k_Thrust   = 1.2e-4; 
prop.k_Torque   = 1.5e-5;  % Front masks
prop.k_torque   = 1.5e-5;  % Rear masks (matching lowercase 't')

% =========================================================================
% COMPONENT DATA
% F-Rotor positions are now defined as the PIVOT AXIS [1.95, y, -0.45]
% R-Rotor positions are the STATIC HUB positions [-2.20, y, -0.65]
% =========================================================================
compData = {
    % Body & Ballast
    'Fuselage',   'fuselage', 1500, [7.50, 2.00, 1.60], [ 0.00,  0.00,  0.00], [0, 0, 0];
    'Ballast',    'box',      250,  [0.60, 0.60, 0.60], [ 3.00,  0.00,  0.00], [0, 0, 0];

    % Wings & Tails
    'Main Wing',  'box',      450,  [1.50, 14.50, 0.20], [-0.50,  0.00, -0.60], [0, 0, 0];
    'L V-Tail',   'box',      90,   [1.40,  2.00, 0.08], [-3.50, -0.71, -0.91], [ 45, 0, 0]; 
    'R V-Tail',   'box',      90,   [1.40,  2.00, 0.08], [-3.50,  0.71, -0.91], [-45, 0, 0]; 
    
    % Booms (Ends at X = 2.05)
    'Boom L1',    'box',      55,   [4.50, 0.20, 0.20], [-0.20, -1.80, -0.45], [0, 0, 0];
    'Boom L2',    'box',      55,   [4.50, 0.20, 0.20], [-0.20, -4.20, -0.45], [0, 0, 0];
    'Boom L3',    'box',      55,   [4.50, 0.20, 0.20], [-0.20, -6.50, -0.45], [0, 0, 0];
    'Boom R1',    'box',      55,   [4.50, 0.20, 0.20], [-0.20,  1.80, -0.45], [0, 0, 0];
    'Boom R2',    'box',      55,   [4.50, 0.20, 0.20], [-0.20,  4.20, -0.45], [0, 0, 0];
    'Boom R3',    'box',      55,   [4.50, 0.20, 0.20], [-0.20,  6.50, -0.45], [0, 0, 0];

    % Front Arms (Visual only for inertia, block handles actual dynamic shift)
    'F-Arm L1',   'box',      5,    [0.40, 0.20, 0.20], [ 2.15, -1.80, -0.55], [0, tilt_angle, 0];
    'F-Arm L2',   'box',      5,    [0.40, 0.20, 0.20], [ 2.15, -4.20, -0.55], [0, tilt_angle, 0];
    'F-Arm L3',   'box',      5,    [0.40, 0.20, 0.20], [ 2.15, -6.50, -0.55], [0, tilt_angle, 0];
    'F-Arm R1',   'box',      5,    [0.40, 0.20, 0.20], [ 2.15,  1.80, -0.55], [0, tilt_angle, 0];
    'F-Arm R2',   'box',      5,    [0.40, 0.20, 0.20], [ 2.15,  4.20, -0.55], [0, tilt_angle, 0];
    'F-Arm R3',   'box',      5,    [0.40, 0.20, 0.20], [ 2.15,  6.50, -0.55], [0, tilt_angle, 0];

    % Front Props (POSITIONS SET TO PIVOT AXIS: X=1.95, Z=-0.45)
    'F-Rotor L1', 'crossprop',25,   [1.60, 0.15, 0.05], [ 1.95, -1.80, -0.45], [0, tilt_angle, 0];
    'F-Rotor L2', 'crossprop',25,   [1.60, 0.15, 0.05], [ 1.95, -4.20, -0.45], [0, tilt_angle, 0];
    'F-Rotor L3', 'crossprop',25,   [1.60, 0.15, 0.05], [ 1.95, -6.50, -0.45], [0, tilt_angle, 0];
    'F-Rotor R1', 'crossprop',25,   [1.60, 0.15, 0.05], [ 1.95,  1.80, -0.45], [0, tilt_angle, 0];
    'F-Rotor R2', 'crossprop',25,   [1.60, 0.15, 0.05], [ 1.95,  4.20, -0.45], [0, tilt_angle, 0];
    'F-Rotor R3', 'crossprop',25,   [1.60, 0.15, 0.05], [ 1.95,  6.50, -0.45], [0, tilt_angle, 0];

    % Rear Props (STATIC HUB POSITIONS)
    'R-Rotor L1', 'crossprop',25,   [1.60, 0.15, 0.05], [-2.20, -1.80, -0.65], [0, 0, 0];
    'R-Rotor L2', 'crossprop',25,   [1.60, 0.15, 0.05], [-2.20, -4.20, -0.65], [0, 0, 0];
    'R-Rotor L3', 'crossprop',25,   [1.60, 0.15, 0.05], [-2.20, -6.50, -0.65], [0, 0, 0];
    'R-Rotor R1', 'crossprop',25,   [1.60, 0.15, 0.05], [-2.20,  1.80, -0.65], [0, 0, 0];
    'R-Rotor R2', 'crossprop',25,   [1.60, 0.15, 0.05], [-2.20,  4.20, -0.65], [0, 0, 0];
    'R-Rotor R3', 'crossprop',25,   [1.60, 0.15, 0.05], [-2.20,  6.50, -0.65], [0, 0, 0];
};
N = size(compData, 1);

% Aero Properties {name, normal_vec, CL0, e, i, CD0, CDa, a0, CM0, CMa, CLa}
aeroData = {
    'Main Wing', [0; 0; -1],                 0.05, 0.9, 0.05, 0.01, 1, 0.05, -0.05, 0, 0.01;
    'L V-Tail',  [0; sind(45); -cosd(45)],   0,    0.8, 0,    0.01, 1, 0,     0,    0, 0.01;
    'R V-Tail',  [0; -sind(45); -cosd(45)],  0,    0.8, 0,    0.01, 1, 0,     0,    0, 0.01;
};

%% =========================================================================
%% 2. MASS, CG, & INERTIA CALCULATIONS
%% =========================================================================
Mtot = 0; CG_row = [0, 0, 0];
for i = 1:N
    m = compData{i, 3}; pos = compData{i, 5};
    Mtot = Mtot + m; CG_row = CG_row + m * pos;
end
CG_row = CG_row / Mtot;

% Set Exact Block Variable Names
Mass = Mtot;
CG   = CG_row'; 

I_total = zeros(3,3); 
for i = 1:N
    type = compData{i, 2}; m = compData{i, 3};
    dim = compData{i, 4}; pos = compData{i, 5};
    eul = compData{i, 6} * (pi/180); 
    
    I_local = zeros(3,3);
    if strcmp(type, 'crossprop')
        D = dim(1); W = dim(2); T = dim(3);
        Ixx1 = (1/12)*(m/2)*(W^2 + T^2); Iyy1 = (1/12)*(m/2)*(D^2 + T^2); Izz1 = (1/12)*(m/2)*(D^2 + W^2);
        Ixx2 = (1/12)*(m/2)*(D^2 + T^2); Iyy2 = (1/12)*(m/2)*(W^2 + T^2); Izz2 = (1/12)*(m/2)*(D^2 + W^2);
        I_local = diag([Ixx1+Ixx2, Iyy1+Iyy2, Izz1+Izz2]);
    else
        xL = dim(1); yL = abs(dim(2)); zL = dim(3);
        I_local = diag([(1/12)*m*(yL^2 + zL^2), (1/12)*m*(xL^2 + zL^2), (1/12)*m*(xL^2 + yL^2)]);
    end
    
    phi = eul(1); theta = eul(2); psi = eul(3);
    Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
    Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
    Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    R = Rz * Ry * Rx; 
    
    I_rot = R * I_local * R'; 
    d = (pos - CG_row)'; 
    I_total = I_total + I_rot + m * ( (d'*d)*eye(3) - (d*d') );
end

% Set Exact Block Variable Name
J = I_total;

%% =========================================================================
%% 3. BUILD STRUCTS (AERO & PROPELLERS)
%% =========================================================================

% --- Build Aero Structs (ABSOLUTE POSITIONS) ---
% Main Wing
wing.c = compData{3, 4}(1); wing.b = compData{3, 4}(2); wing.S = wing.b * wing.c;
wing.half_rho_S = 0.5 * rho * wing.S; wing.pos = (compData{3, 5})'; 
wing.n = aeroData{1, 2}; wing.CL0 = aeroData{1, 3}; wing.e = aeroData{1, 4};
wing.i = aeroData{1, 5}; wing.CD0 = aeroData{1, 6}; wing.CDa = aeroData{1, 7};
wing.a0 = aeroData{1, 8}; wing.CM0 = aeroData{1, 9}; wing.CMa = aeroData{1, 10};
wing.CLa = aeroData{1, 11}; wing.AR = wing.b / wing.c;

% Left Tail
tailL.c = compData{4, 4}(1); tailL.b = compData{4, 4}(2); tailL.S = tailL.b * tailL.c;
tailL.half_rho_S = 0.5 * rho * tailL.S; tailL.pos = (compData{4, 5})';
tailL.n = aeroData{2, 2}; tailL.CL0 = aeroData{2, 3}; tailL.e = aeroData{2, 4};
tailL.i = aeroData{2, 5}; tailL.CD0 = aeroData{2, 6}; tailL.CDa = aeroData{2, 7};
tailL.a0 = aeroData{2, 8}; tailL.CM0 = aeroData{2, 9}; tailL.CMa = aeroData{2, 10};
tailL.CLa = aeroData{2, 11}; tailL.AR = tailL.b / tailL.c;

% Right Tail
tailR.c = compData{5, 4}(1); tailR.b = compData{5, 4}(2); tailR.S = tailR.b * tailR.c;
tailR.half_rho_S = 0.5 * rho * tailR.S; tailR.pos = (compData{5, 5})';
tailR.n = aeroData{3, 2}; tailR.CL0 = aeroData{3, 3}; tailR.e = aeroData{3, 4};
tailR.i = aeroData{3, 5}; tailR.CD0 = aeroData{3, 6}; tailR.CDa = aeroData{3, 7};
tailR.a0 = aeroData{3, 8}; tailR.CM0 = aeroData{3, 9}; tailR.CMa = aeroData{3, 10};
tailR.CLa = aeroData{3, 11}; tailR.AR = tailR.b / tailR.c;

% --- Build Propeller Position Matrices ---
idx_FR = find(contains(compData(:,1), 'F-Rotor R'));
idx_FL = find(contains(compData(:,1), 'F-Rotor L'));
idx_RR = find(contains(compData(:,1), 'R-Rotor R'));
idx_RL = find(contains(compData(:,1), 'R-Rotor L'));

prop.posFR = cell2mat(compData(idx_FR, 5));
prop.posFL = cell2mat(compData(idx_FL, 5));
prop.posRR = cell2mat(compData(idx_RR, 5));
prop.posRL = cell2mat(compData(idx_RL, 5));

%% =========================================================================
%% 4. TEST CASE MANAGER
%% =========================================================================

% --- Control Inputs (Red blocks in Propellers subsystem) ---
Motor_RPMs = zeros(12, 1);    
Tilt_angles = zeros(6, 1);    

test_case = 'Stable_6DOF'; % Change this to run different cases

switch test_case
    case 'Case1_UnstableAxis'
        pos_init   = [0; 0; -1000];
        V_init     = [10; 0; 0];
        eul_init   = [0; 10; 0] * (pi/180); % Converted to rad
        omega_init = [0; 0; 0];
        g          = 0; % Gravity off for tester cases [cite: 71]
        
    case 'Case2_OutsideLoop'
        pos_init   = [0; 0; -1000];
        V_init     = [13; 0; 0]; 
        eul_init   = [0; 0; 0]; 
        omega_init = [0; 1; 0]; 
        g          = 0; 
        
    case 'Case3_ForceField'
        pos_init   = [0; 0; -1000];
        V_init     = [13; 0; 0.25]; 
        eul_init   = [0; 0; 0]; 
        omega_init = [0; 0; 0]; 
        g          = -9.81; 

    case 'Stable_6DOF'
        % Final Validation: Stable 6DOF Flight
        pos_init   = [0; 0; -1000]; 
        V_init     = [13; 0.25; 0.25]; 
        eul_init   = [5; 5; 0] * (pi/180); % Example <[10;10;any] [cite: 69]
        omega_init = [0.005; 0.5; 0.005];  % Example <[1e-2; 1; 1e-2] [cite: 69]
        g          = -9.81;
        
        % Turn on cruise prop RPMs/Tilts
        Motor_RPMs(:) = 1500; 
        Tilt_angles(:) = 90;
end


%% =========================================================================
%% 5. 3D VISUALIZATION
%% =========================================================================

disp('Rendering 3D Aircraft Model...');

figure('Color',[0.1 0.1 0.1],'Position',[50 50 1400 900]); 
hold on; grid on; axis equal;
set(gca, 'ZDir', 'reverse', 'Color', [0.15 0.15 0.15], 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w'); 
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');

% Format title based on flight mode (numeric or string)
if isnumeric(flight_mode)
    mode_str = ['Mode: ', num2str(flight_mode), ' (Tilt: ', num2str(tilt_angle), '°)'];
else
    mode_str = sprintf('Mode: %s (Tilt: %d°)', upper(flight_mode), tilt_angle);
end
title(mode_str, 'Color', 'w', 'FontSize', 14);

% Pre-calculate thrust vector for the front nacelle offsets
n_thrust = [sind(tilt_angle), 0, -cosd(tilt_angle)]; 
arm_offset = 0.30; % Visual center of the standoff arm

N = size(compData, 1);
for i = 1:N
    name = compData{i, 1}; 
    type = compData{i, 2};
    dim  = compData{i, 4}; 
    pos  = compData{i, 5}; 
    eul  = compData{i, 6} * (pi/180);
    
    % --- Dynamic Position Interception for Front Rotors/Arms ---
    % Because compData holds the PIVOT position for F-Rotors to keep inertia
    % math clean, we visually shift them out to the hub here.
    if contains(name, 'F-Arm')
        % Override dimensions to exact standoff size and shift out by arm_offset
        dim = [0.20, 0.20, 0.40];
        pos = [1.95, pos(2), -0.45] + (arm_offset * n_thrust);
        eul = [0, tilt_angle, 0] * (pi/180);
    elseif contains(name, 'F-Rotor')
        % Shift rotor hub out from pivot by the exact hub_offset
        pos = [1.95, pos(2), -0.45] + (prop.hub_offset * n_thrust);
        eul = [0, tilt_angle, 0] * (pi/180);
    end

    % --- Set Colors ---
    if strcmp(type, 'fuselage'); color = [0.0 1.0 1.0]; 
    elseif contains(name, 'Ballast'); color = [1.0 1.0 0.0]; 
    elseif contains(name, 'Wing'); color = [1.0 0.0 1.0]; 
    elseif contains(name, 'Tail'); color = [1.0 0.5 0.0]; 
    elseif strcmp(type, 'box'); color = [0.0 1.0 0.0]; 
    elseif contains(name, 'F-Rotor'); color = [1.0 0.0 0.0]; 
    elseif contains(name, 'R-Rotor'); color = [0.0 0.5 1.0]; 
    end
    
    % --- Generate Geometry ---
    if strcmp(type, 'fuselage')
        [V, F] = genFacetedFuselage(dim(1), dim(2), dim(3));
    elseif strcmp(type, 'box')
        [V, F] = genCleanBox(dim(1), dim(2), dim(3));
    elseif strcmp(type, 'crossprop')
        [V, F] = genCrossProp(dim(1), dim(2), dim(3)); 
    end
    
    % --- Rotate and Translate ---
    phi = eul(1); theta = eul(2); psi = eul(3);
    Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
    Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
    Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    
    V_trans = ((Rz * Ry * Rx) * V')' + pos; 
    
    patch('Vertices', V_trans, 'Faces', F, ...
        'FaceColor', color, 'FaceAlpha', 0.85, 'EdgeColor', 'k', 'LineWidth', 1.0);
end

% CG Marker
plot3(CG(1), CG(2), CG(3), 'wp', 'MarkerFaceColor','w', 'MarkerSize',15, 'LineWidth', 2);
text(CG(1), CG(2), CG(3)-0.6, '  CG', 'FontWeight','bold', 'FontSize', 12, 'Color', 'w');

% --- VERIFICATION MARKERS (BLOCK INPUTS) ---
disp('Plotting verification markers for block inputs...');

% 1. Aero Block Inputs (Magenta Triangles)
% wing.pos, tailL.pos, tailR.pos are 3x1 column vectors
plot3(wing.pos(1), wing.pos(2), wing.pos(3), 'm^', 'MarkerSize', 10, 'MarkerFaceColor', 'm');
plot3(tailL.pos(1), tailL.pos(2), tailL.pos(3), 'm^', 'MarkerSize', 10, 'MarkerFaceColor', 'm');
plot3(tailR.pos(1), tailR.pos(2), tailR.pos(3), 'm^', 'MarkerSize', 10, 'MarkerFaceColor', 'm');

% 2. Propeller Block Inputs (Cyan Circles)
% prop.posFR, posFL, posRR, posRL are 3x3 matrices (Rows = rotors, Cols = X, Y, Z)
% Front ones verify the Pivot inputs; Rear ones verify the Hub inputs
plot3(prop.posFR(:,1), prop.posFR(:,2), prop.posFR(:,3), 'co', 'MarkerSize', 8, 'LineWidth', 2);
plot3(prop.posFL(:,1), prop.posFL(:,2), prop.posFL(:,3), 'co', 'MarkerSize', 8, 'LineWidth', 2);
plot3(prop.posRR(:,1), prop.posRR(:,2), prop.posRR(:,3), 'co', 'MarkerSize', 8, 'LineWidth', 2);
plot3(prop.posRL(:,1), prop.posRL(:,2), prop.posRL(:,3), 'co', 'MarkerSize', 8, 'LineWidth', 2);

% Optional: Add a legend just for these markers so you remember what they are
% (Requires grabbing the plot handles)
hAero = plot3(NaN, NaN, NaN, 'm^', 'MarkerSize', 10, 'MarkerFaceColor', 'm');
hProp = plot3(NaN, NaN, NaN, 'co', 'MarkerSize', 8, 'LineWidth', 2);
hCG   = plot3(NaN, NaN, NaN, 'wp', 'MarkerFaceColor','w', 'MarkerSize',15);
legend([hCG, hAero, hProp], {'Center of Gravity', 'Aero Block Inputs', 'Prop Block Inputs'}, 'TextColor', 'w', 'Color', 'none', 'Location', 'best');

view(130, 25); camlight right; lighting flat; hold off;


%% =========================================================================
%% GEOMETRY HELPER FUNCTIONS
%% =========================================================================
function [V, F] = genCleanBox(L, W, H)
    x = L/2; y = W/2; z = H/2;
    V = [-x -y -z;  x -y -z;  x  y -z; -x  y -z; 
         -x -y  z;  x -y  z;  x  y  z; -x  y  z];
    F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
end

function [V, F] = genCrossProp(D, W, T)
    [V1, F1] = genCleanBox(D, W, T);
    [V2, F2] = genCleanBox(W, D, T);
    V = [V1; V2];
    F = [F1; F2 + 8]; 
end

function [V, F] = genFacetedFuselage(L, W, H)
    x_norm = [0.5, 0.35, 0.05, -0.3, -0.5];
    w_norm = [0.3, 0.8, 1.0, 0.6, 0.2];
    z_top_norm = [-0.2, -0.7, -1.0, -0.6, -0.3];
    z_bot_norm = [ 0.4,  0.8,  1.0,  0.5, -0.1];
    n_sec = length(x_norm); V = [];
    for i = 1:n_sec
        x = x_norm(i) * L; y = w_norm(i) * W / 2;
        zT = z_top_norm(i) * H / 2; zB = z_bot_norm(i) * H / 2;
        V = [V; x, y, zT; x, -y, zT; x, -y, zB; x, y, zB];
    end
    F = [];
    for i = 1:(n_sec-1)
        idx = (i-1)*4;
        F = [F; idx+1, idx+2, idx+6, idx+5]; F = [F; idx+2, idx+3, idx+7, idx+6]; 
        F = [F; idx+3, idx+4, idx+8, idx+7]; F = [F; idx+4, idx+1, idx+5, idx+8]; 
    end
    F = [F; 1, 2, 3, 4]; idx = (n_sec-1)*4; F = [F; idx+4, idx+3, idx+2, idx+1]; 
end