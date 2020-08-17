% Update parameter based on selected primitives (or inputs).
function [s_mu,s_sigma] = ce_param_update_mex(s_mu,s_sigma,...
    ce_param_m,ce_param_dim,ce_param_alpha,ce_param_beta,ce_param_b_q,Z_el,iter)

s_pre_mu        = s_mu;
s_pre_sigma     = s_sigma;

for j = 1:ce_param_m
    ind_1x2         = (j-1) * ce_param_dim + (1:ce_param_dim); % ind = [2*j-1, 2*j]; [1, 2] [3, 4] ...
    s_mu(ind_1x2)   = mean(Z_el(:,ind_1x2));
    s_sigma(ind_1x2, ind_1x2) = cov(Z_el(:,ind_1x2));
end

s_mu    = ce_param_alpha * s_mu + (1-ce_param_alpha) * s_pre_mu;
b_mod   = ce_param_beta - ce_param_beta * (1-1/iter)^ce_param_b_q;
s_sigma = b_mod * s_sigma + (1-b_mod) * s_pre_sigma;
end
