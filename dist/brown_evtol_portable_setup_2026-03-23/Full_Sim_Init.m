%% =========================================================================
%% 1. INITIALIZATION & AIRCRAFT PARAMETERS
%% =========================================================================
clear; close all; clc;

% Keep the setup portable when these files are shared outside the repo.
% Full_Sim_Init.m, aircraft_def.m, scenario_def.m, and render_aircraft.m
% should live in the same folder.
stack = dbstack('-completenames');
if ~isempty(stack)
    script_dir = fileparts(stack(1).file);
else
    script_dir = pwd;
end
addpath(script_dir);

required_files = {'aircraft_def.m', 'scenario_def.m', 'render_aircraft.m'};
for idx = 1:numel(required_files)
    required_path = fullfile(script_dir, required_files{idx});
    if exist(required_path, 'file') ~= 2
        error('Full_Sim_Init:MissingDependency', ...
            ['Missing %s in %s.\n' ...
             'Keep Full_Sim_Init.m, aircraft_def.m, scenario_def.m, and render_aircraft.m together when sharing.'], ...
            required_files{idx}, script_dir);
    end
end

% Change this to run different cases.
test_case = 'Stable_6DOF';

% The aircraft definition keeps the legacy reference geometry at hover.
% Visualization uses the scenario tilt commands instead.
aircraft = aircraft_def('flight_mode', 0);
scenario = scenario_def(test_case);

% Export the same workspace variables that the Simulink models expect.
flight_mode = scenario.flight_mode;
structural_tilt_angle = aircraft.tilt_angle;
tilt_angle = scenario.visual_tilt_deg;

rho = aircraft.rho;
g = scenario.g;

prop = aircraft.prop;
compData = aircraft.compData;
aeroData = aircraft.aeroData;

Mass = aircraft.Mass;
CG = aircraft.CG;
J = aircraft.J;

wing = aircraft.wing;
wingL = aircraft.wingL;
wingR = aircraft.wingR;
tailL = aircraft.tailL;
tailR = aircraft.tailR;

pos_init = scenario.pos_init;
V_init = scenario.V_init;
eul_init = scenario.eul_init;
omega_init = scenario.omega_init;
Motor_RPMs = scenario.Motor_RPMs;
Tilt_angles = scenario.Tilt_angles;
Fext_B = scenario.Fext_B;
Mext_B = scenario.Mext_B;

%% =========================================================================
%% 2. 3D VISUALIZATION
%% =========================================================================
disp('Rendering 3D Aircraft Model...');

render_aircraft(compData, CG, tilt_angle, ...
    'surfaces', aircraft.render_surfaces, ...
    'prop', prop, ...
    'thrust_tilt_deg', Tilt_angles, ...
    'title_text', sprintf('%s (Scenario: %s)', 'Brown eVTOL Aircraft', test_case));
