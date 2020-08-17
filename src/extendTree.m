% Extend tree (steer function).
% cp: current point
% tp: target(sampled) point
% R : dynamic info (struct)
%
% pseg: path segment
% u: selected input (linear_speed, angular_speed, ntime)
function [pseg,u] = extendTree(cp,tp,R)

% dimension
dim_x       = R.dim_x;
dim_u       = R.dim_u;

% time
dt          = R.dt;
tN          = R.tN;
delta_t     = dt*tN;
dt_tmp      = 0:dt:delta_t;

u_range     = R.u_range;

% difference & distance
diff        = tp(1:2) - cp(1:2);

switch R.dynamic
    case 'pointmass'
        u_tmp   = diff/delta_t;
        u_tmp   = max(u_tmp,u_range(:,1));
        u_tmp   = min(u_tmp,u_range(:,2));
        
        u       = [repmat(u_tmp,1,tN); dt*ones(1,tN)];
        pseg    = repmat(cp,1,tN+1) + repmat(dt_tmp,dim_u,1).*[zeros(dim_u,1) u(1:dim_u,:)];
        
    case 'unicycle_part'
        
        u_linear = R.u_linear;
        
        % pop out N random inputs
        N = 25;
        w_array         = u_range(2,1) + (u_range(2,2) - u_range(2,1))*rand(1,N);
        NT              = tN + 1;
        path_array      = zeros(dim_x*N,NT);
        endindex_array  = zeros(1,N);
        dist_array      = zeros(1,N);
        
        for nidx_n = 1:1:N
            v   = u_linear;
            w   = w_array(1,nidx_n);
            traj = zeros(dim_x,size(dt_tmp,2));
            traj(:,1) = cp;
            for nidx_t = 2:1:(size(dt_tmp,2))
                theta   = traj(3,nidx_t-1);
                if(w == 0)
                    traj(1:3,nidx_t) = traj(1:3,nidx_t-1)+[v*cos(theta)*dt; v*sin(theta)*dt; 0];
                else
                    traj(1:3,nidx_t) = traj(1:3,nidx_t-1)+[-v/w*sin(theta)+v/w*sin(theta+w*dt);...
                        v/w*cos(theta)-v/w*cos(theta+w*dt);...
                        w*dt];
                end
            end
            traj(3,nidx_t) = angle_handle(traj(3,nidx_t));
            
            idx_pa  = ((nidx_n-1)*dim_x + 1):1:(nidx_n*dim_x);
            path_array(idx_pa,:) = traj;
            
            diff_tmp    = repmat(tp,1,size(traj,2)) - traj(R.idx_ws,:);
            dist_sq_tmp = sum(diff_tmp.*diff_tmp,1);
            [dist_min,idx_min]          = min(dist_sq_tmp);
            endindex_array(1,nidx_n)    = idx_min;
            dist_array(1,nidx_n)        = dist_min;
        end
        
        [~,index_min]   = min(dist_array);
        end_index       = endindex_array(1,index_min);
        
        idx_pa_min  = ((index_min-1)*dim_x + 1):1:(index_min*dim_x);
        pseg        = path_array(idx_pa_min,1:end_index);
        w           = w_array(:,index_min);
        ntime       = end_index;
        
        if(ntime>1)
            u = repmat([u_linear; w; dt],1,ntime-1);
        else
            u = [];
        end
end % switch
end
