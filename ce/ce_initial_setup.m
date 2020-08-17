function s = ce_initial_setup(sp,gp,R,ce_param,sinit_sigma)

% set initial primitives
switch R.dynamic
    case 'pointmass'
        % create line connecting two points (sp, gp); linearized sampling
        xs = zeros(2,ce_param.m + 2);
        xs(1,:) = linspace(sp(1),gp(1), ce_param.m + 2);
        xs(2,:) = linspace(sp(2),gp(2), ce_param.m + 2);
    case 'unicycle_part'
        % create control set of unicycle dynamic (time, angle-velocity)
        min_ti = norm(gp(1:2)-sp(1:2))/R.u_linear;     % minimum time of arriving at g_p
        z = repmat([min_ti/ce_param.m,0],1,ce_param.m);     %initial input time and control
    case 'unicycle_full'
        % create control set of unicycle dynamic (time, angle-velocity)
        min_ti = norm(gp(1:2)-sp(1:2))/R.u_linear;     % minimum time of arriving at g_p
        z = repmat([min_ti/ce_param.m,R.u_linear,0],1,ce_param.m);     %initial input time and control
end

% update mu & sigma
switch R.dynamic
    case 'pointmass'
        s.mu = reshape(xs(:,2:end-1), ce_param.dim*ce_param.m, 1);
        for nidx_m = 1:1:ce_param.m
            s.sigma(2*nidx_m-1:2*nidx_m,2*nidx_m-1:2*nidx_m) = sinit_sigma;
        end
    case 'unicycle_part'
        s.mu = z;
        s.sigma = diag(repmat(sinit_sigma,1,ce_param.m));
end
end
