function results = diagnose_longitudinal_drift()
%DIAGNOSE_LONGITUDINAL_DRIFT Compare simple cases for the current wrapper.

repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
origDir = pwd;
cleanupDir = onCleanup(@() cd(origDir)); %#ok<NASGU>
cd(repoRoot);

addpath(genpath(fullfile(repoRoot, 'scripts')));

results = struct();
results.baseline = localRunCase(1, 30, 30);
results.noKi = localRunCase(1, 0, 30);
results.noNoise = localRunCase(0, 30, 30);

disp('BASELINE')
disp(struct2table(results.baseline))

disp('NO_KI')
disp(struct2table(results.noKi))

disp('NO_NOISE')
disp(struct2table(results.noNoise))
end

function stats = localRunCase(noiseFlag, KiVal, stopTime)
run('scripts/control/setup_eul_controller_demo.m');

NoiseOn = noiseFlag; %#ok<NASGU>
gains.V.Ki = KiVal; %#ok<NASGU>

cmd_profile.vinf.enable = true; %#ok<NASGU>
cmd_profile.vinf.time = 3; %#ok<NASGU>
cmd_profile.vinf.magnitude = 5; %#ok<NASGU>
cmd_profile.phi.enable = false; %#ok<NASGU>
cmd_profile.theta.enable = false; %#ok<NASGU>

simOut = sim('Brown_6DOF_Sim_Wrapper', ...
    'StopTime', num2str(stopTime), ...
    'ReturnWorkspaceOutputs', 'on');

    vinf = simOut.get('vinf_truth');
    eul = simOut.get('eul_truth');
    plantCmd = simOut.get('plant_cmd_bus');

    t = vinf.time(:);
    V = vinf.signals.values(:);
    theta = rad2deg(eul.signals.values(:,2));

    vals = squeeze(plantCmd.signals.values);
    if size(vals,1) < size(vals,2)
        vals = vals.';
    end

    frontCollective = vals(:,19);
    deltaE = rad2deg(vals(:,23));

    beforeIdx = find(t < 3, 1, 'last');
    if isempty(beforeIdx)
        beforeIdx = 1;
    end

    stats = struct();
    stats.v_before = V(beforeIdx);
    stats.v_end = V(end);
    stats.v_max = max(V);
    stats.theta_before_deg = theta(beforeIdx);
    stats.theta_end_deg = theta(end);
    stats.theta_max_deg = max(theta);
    stats.front_before_rpm = frontCollective(beforeIdx);
    stats.front_end_rpm = frontCollective(end);
    stats.front_max_rpm = max(frontCollective);
    stats.delta_e_end_deg = deltaE(end);
end
