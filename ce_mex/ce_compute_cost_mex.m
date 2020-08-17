% Compute cost.
function [cost_terminal,cost_traj] = ce_compute_cost_mex(traj,gp,...
    R_dist_w,R_idx_ws,R_dim_ws,map_X,map_Y,map_C,cost_type,interval_cost)
% cost_terminal : distance between goal point between end of the trajectory
% cost_traj     : cost of trajectory 'distance' and 'environment map'

if(size(gp,1) == size(traj,1))
    cost_terminal = norm(gp(:,1)-traj(:,end));
elseif(size(gp,1) == R_dim_ws)
    cost_terminal = norm(gp(:,1)-traj(R_idx_ws,end));
else
    cost_terminal = 0;
end


switch cost_type
    case 1
        diff    = traj(:,2:end) - traj(:,1:end-1);
        dist    = sqrt(sum(R_dist_w*(diff.*diff),2));
        cost_tmp = dist;
        
    case 2
        interval_cost = max(0.2,interval_cost);
        
        % extend trajectory
        traj_extended = make_denseTrajectory(traj(R_idx_ws,:),interval_cost,R_dim_ws);
        
        Xq = traj_extended(1,:);
        Yq = traj_extended(2,:);
        Zq = interp2(map_X,map_Y,map_C,Xq,Yq);
        cost_tmp = sum(Zq,2);
        
    otherwise
        cost_tmp = 0;
end
cost_traj = cost_tmp;
end
