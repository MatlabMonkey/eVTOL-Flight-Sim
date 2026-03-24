function scenario = scenario_def(test_case)
%SCENARIO_DEF Define initial conditions and open-loop commands for a run.

if nargin < 1 || isempty(test_case)
    test_case = 'Stable_6DOF';
end

Motor_RPMs = zeros(12, 1);
Tilt_angles = zeros(6, 1);
Fext_B = [0; 0; 0];
Mext_B = [0; 0; 0];

switch char(test_case)
    case 'Case1_UnstableAxis'
        pos_init = [0; 0; -1000];
        V_init = [10; 0; 0];
        eul_init = [0; 10; 0] * (pi / 180);
        omega_init = [0; 0; 0];
        g = 0;

    case 'Case2_OutsideLoop'
        pos_init = [0; 0; -1000];
        V_init = [13; 0; 0];
        eul_init = [0; 0; 0];
        omega_init = [0; 1; 0];
        g = 0;

    case 'Case3_ForceField'
        pos_init = [0; 0; -1000];
        V_init = [13; 0; 0.25];
        eul_init = [0; 0; 0];
        omega_init = [0; 0; 0];
        g = -9.81;

    case 'Stable_6DOF'
        pos_init = [0; 0; -1000];
        V_init = [13; 0.25; 0.25];
        eul_init = [5; 5; 0] * (pi / 180);
        omega_init = [0.005; 0.5; 0.005];
        g = -9.81;

        Motor_RPMs(:) = 1500;
        Tilt_angles(:) = 90;

    otherwise
        error('scenario_def:UnknownCase', ...
            'Unknown test case ''%s''.', char(test_case));
end

visual_tilt_deg = localRepresentativeFrontTilt(Tilt_angles);

scenario = struct();
scenario.name = char(test_case);
scenario.pos_init = pos_init;
scenario.V_init = V_init;
scenario.eul_init = eul_init;
scenario.omega_init = omega_init;
scenario.g = g;
scenario.Motor_RPMs = Motor_RPMs;
scenario.Tilt_angles = Tilt_angles;
scenario.Fext_B = Fext_B;
scenario.Mext_B = Mext_B;
scenario.visual_tilt_deg = visual_tilt_deg;
scenario.flight_mode = double(visual_tilt_deg >= 45);
end

function visual_tilt_deg = localRepresentativeFrontTilt(Tilt_angles)
if isempty(Tilt_angles)
    visual_tilt_deg = 0;
    return;
end

visual_tilt_deg = mean(Tilt_angles(:));
end
