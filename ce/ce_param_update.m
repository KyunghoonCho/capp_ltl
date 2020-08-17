function s = ce_param_update(s,ce_params,Z_el,iter)
% Update parameter based on selected primitives (or inputs).

s_pre.mu = s.mu;
s_pre.sigma = s.sigma;

for j = 1:ce_params.m
    ind_1x2 = (j-1) * ce_params.dim + (1:ce_params.dim); % ind = [2*j-1, 2*j]; [1, 2] [3, 4] ...
    s.mu(ind_1x2) = mean(Z_el(:,ind_1x2));
    s.sigma(ind_1x2, ind_1x2) = cov(Z_el(:,ind_1x2));
end

s.mu = ce_params.alpha * s.mu + (1-ce_params.alpha) * s_pre.mu;
b_mod = ce_params.beta - ce_params.beta * (1-1/iter)^ce_params.b_q;
s.sigma = b_mod * s.sigma + (1-b_mod) * s_pre.sigma;
end
