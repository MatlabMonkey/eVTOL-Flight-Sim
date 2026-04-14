function y = trim_residual_component(u, idx)
%TRIM_RESIDUAL_COMPONENT Return one scalar component of trim_residual_eval.
residual = trim_residual_eval(u);
y = residual(idx);
end
