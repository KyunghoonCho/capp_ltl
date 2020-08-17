function [pseg, useg] = ce_generate_point_path(sp,gp,z,m,dist_interval,R,use_mex)
% Generate trajectory from primitive

dim_x = R.dim_x;
dim_u = R.dim_u;

traj_tmp = [sp,reshape(z,dim_x,m),gp];

% extend trajectory
if use_mex
    pseg = make_denseTrajectory_mex(traj_tmp,dist_interval,dim_x);
else
    pseg = make_denseTrajectory(traj_tmp,dist_interval,dim_x);

% generate ctrl
useg = zeros(dim_u+1,4000);
idx_u = 0;
for nidx_t = 1:1:size(pseg,2)
    diff    = gp(:,1) - sp(:,1);
    tmax    = max(abs(diff./R.u_range(:,2)));
    u_tmp   = diff/tmax;
    u       = [u_tmp; tmax];
    
    idx_u = idx_u + 1;
    useg(:,idx_u) = u;
end
useg = useg(:,1:idx_u);

end
