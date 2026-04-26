function u_cmd = controller_hold_trim(trim_cmd)
%CONTROLLER_HOLD_TRIM Pass the trim command straight through.

u_cmd = zeros(6, 1);
count = min(numel(trim_cmd), 6);
if count > 0
    u_cmd(1:count) = trim_cmd(1:count);
end
end
