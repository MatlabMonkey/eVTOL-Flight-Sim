function gains = apply_default_controller_gains(varargin)
%APPLY_DEFAULT_CONTROLLER_GAINS Set a first-pass cruise controller gain set.

p = inputParser;
p.addParameter('Profile', 'cruise_v1', @(x) ischar(x) || isstring(x));
p.parse(varargin{:});

profile = lower(string(p.Results.Profile));
switch profile
    case "cruise_v1"
        gains = struct( ...
            'k_p_damp', 1.0, ...
            'k_q_damp', 4.0, ...
            'k_r_damp', 1.0, ...
            'k_phi_p', 0.4, ...
            'k_phi_i', 0.0, ...
            'k_v_p', 5.0, ...
            'k_v_i', 0.0);
    otherwise
        error('apply_default_controller_gains:UnknownProfile', ...
            'Unknown controller profile: %s', profile);
end

assignin('base', 'controller_k_p_damp', gains.k_p_damp);
assignin('base', 'controller_k_q_damp', gains.k_q_damp);
assignin('base', 'controller_k_r_damp', gains.k_r_damp);
assignin('base', 'controller_k_phi_p', gains.k_phi_p);
assignin('base', 'controller_k_phi_i', gains.k_phi_i);
assignin('base', 'controller_k_v_p', gains.k_v_p);
assignin('base', 'controller_k_v_i', gains.k_v_i);
end
