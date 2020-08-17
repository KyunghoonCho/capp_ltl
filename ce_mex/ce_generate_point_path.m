% Generate trajectory from primitive.
function [pseg, useg] = ce_generate_point_path(sp,gp,z,m,dim_x,dim_u,u_range,dist_interval)

traj_tmp = [sp,reshape(z,dim_x,m),gp];

% extend trajectory
pseg = make_denseTrajectory(traj_tmp,dist_interval,dim_x);

% generate ctrl
useg = zeros(dim_u+1,4000);
idx_u = 0;
for nidx_t = 2:1:size(pseg,2)
    diff    = pseg(:,nidx_t) - pseg(:,nidx_t-1);
    tmax    = max(abs(diff./u_range(:,2)));
    u_tmp   = diff/tmax;
    u       = [u_tmp; tmax];
    
    idx_u = idx_u + 1;
    useg(:,idx_u) = u;
end
useg = useg(:,1:idx_u);

end
