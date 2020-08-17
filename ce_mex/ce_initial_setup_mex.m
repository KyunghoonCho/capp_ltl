function [s_mu,s_sigma] = ce_initial_setup_mex(sp,gp,R_dynamic,R_ulinear,...
    ce_param_m,ce_param_dim,sinit_sigma)

switch R_dynamic
    case 1 % 'pointmass'
        % set initial primitives
        % create line connecting two points (sp, gp); linearized sampling
        xs      = zeros(2, ce_param_m + 2);
        xs(1,:) = linspace(sp(1),gp(1), ce_param_m + 2);
        xs(2,:) = linspace(sp(2),gp(2), ce_param_m + 2);
        
        % update mu & sigma
        s_mu = reshape(xs(:,2:end-1), ce_param_dim*ce_param_m, 1);
        s_sigma = zeros(2*ce_param_m,2*ce_param_m);
        for nidx_m = 1:1:ce_param_m
            s_sigma(2*nidx_m-1:2*nidx_m,2*nidx_m-1:2*nidx_m) = sinit_sigma;
        end
        
    case 2 % 'unicycle_part'
        % set initial primitives
        % create control set of unicycle dynamic (time, angle-velocity)
        min_ti = norm(gp(1:2)-sp(1:2))/R_ulinear;           % minimum time of arriving at g_p
        z = repmat([min_ti/ce_param_m,0],1,ce_param_m);     %initial input time and control
        
        % update mu & sigma
        s_mu    = z;
        
        if(size(sinit_sigma,1)==1)
            s_sigma = diag(repmat(sinit_sigma(1,:),1,ce_param_m));
        elseif(size(sinit_sigma,2)==1)
            s_sigma = diag(repmat(sinit_sigma(:,1)',1,ce_param_m));
        else
            s_sigma = sinit_sigma;
        end
        
    case 3 % 'unicycle_full'
        % create control set of unicycle dynamic (time, angle-velocity)
        min_ti = norm(gp(1:2)-sp(1:2))/R_ulinear;           % minimum time of arriving at g_p
        z = repmat([min_ti/ce_param_m,R_ulinear,0],1,ce_param_m);     %initial input time and control
        
        % temporal issue ...
        s_mu = zeros(1,2);
        s_sigma = zeros(2,2);
    otherwise
        s_mu = zeros(1,2);
        s_sigma = zeros(2,2);
end

end
