function [traj,ctrl,flag] = ce_generate_trajectory(sp,z,dim,m,boundary,u_linear,dt)
% Generate trajectory from primitive. (from input)

% Input
%   z: primitive (or input)
%   dim: dimension of primitive
%       2 : unicycle dynamic (only time and w are sampled and v is fixed)
%       3 : unicycle dynamic (ver2, time, v, and w are sampled)
%   m: number of primitive
%   boundary
%   u_linear,dt

% initialize trajectory
traj                = zeros(3,m*200);
idx_t               = 1;
traj(1:3,idx_t)     = sp(1:3,1);

ctrl                = zeros(3,m*200);
idx_c               = 0;

flag                = 0;

switch dim
    case 2
        t = z(1:dim:m*dim);
        v = u_linear * ones(1,m);
        w = z(2:dim:m*dim);
    case 3
        t = z(1:dim:m*dim);
        v = z(2:dim:m*dim);
        w = z(3:dim:m*dim);
    otherwise
        t = z(1:dim:m*dim);
        v = z(2:dim:m*dim);
        w = z(3:dim:m*dim);
end

N_t         = 10;
t_rounded   = round(t*N_t)/N_t;
for nidx_m = 1:m
    tau         = t_rounded(nidx_m);
    tau_s       = [0 dt : dt :tau];
    u_linear    = v(nidx_m);
    u_angular   = w(nidx_m);
    
    if tau == 0
        continue;
    end
    
    for nidx_t = 1:1:(size(tau_s,2)-1)
        theta       = traj(3,idx_t);
        dt_sel = tau_s(1,nidx_t+1) - tau_s(1,nidx_t);
        if abs(u_angular)< 1e-10    % when input(u) is neglegible
            traj(1,idx_t+1) = traj(1,idx_t) + dt_sel * u_linear * cos(theta);
            traj(2,idx_t+1) = traj(2,idx_t) + dt_sel * u_linear * sin(theta);
            traj(3,idx_t+1) = traj(3,idx_t);
        else
            traj(1,idx_t+1) = traj(1,idx_t) ...
                + u_linear/u_angular * ( sin(theta + dt_sel * u_angular) - sin(theta));
            traj(2,idx_t+1) = traj(2,idx_t) + ...
                + u_linear/u_angular * ( cos(theta ) - cos(theta + dt_sel * u_angular));
            traj(3,idx_t+1) = traj(3,idx_t) + dt_sel * u_angular;
        end
        
        % constraint check
        if ~((traj(1,idx_t+1)>=boundary(1,1))&&(traj(1,idx_t+1)<=boundary(1,2)))
            flag = 1;
        end
        if ~((traj(2,idx_t+1)>=boundary(2,1))&&(traj(2,idx_t+1)<=boundary(2,2)))
            flag = 1;
        end
        if flag == 1
            break;
        end
        
        % theta in [-pi pi]
        if(traj(3,idx_t+1)>pi)
            traj(3,idx_t+1) = traj(3,idx_t+1) - 2*pi;
        end
        if(traj(3,idx_t+1)<-pi)
            traj(3,idx_t+1) = traj(3,idx_t+1) + 2*pi;
        end
        
        ctrl(:,idx_c+1) = [u_linear; u_angular; dt];
        
        idx_t = idx_t+1;
        idx_c = idx_c+1;
    end
    
    if flag == 1
        break;
    end
end

traj = traj(:,1:idx_t);
ctrl = ctrl(:,1:idx_c);
end
