%% Run Cross-Entropy Randomized Motion Planning
function [path_final,input_final,z_final,cost_final,flag] = CE_MP(sp,gp,R,param,map,sinit,use_mex)

flag = 1;

%% set parameters
N_iter = 100;     % number of iteration 50
traj_num = 50;   % number of trajectory 50
N_el1 = round(traj_num * 0.2);
N_el2 = round(traj_num * 0.1);

% parameters related with update
ce_param.alpha  = .9;
ce_param.beta   = .7;
ce_param.b_q    = 15;

switch R.dynamic
    case 'pointmass'
        % state sampling
        ce_param.m = R.num_elite_position;
        ce_param.dim = 2 * 1;
    case 'unicycle_part'
        % trajectory is parameterized by set of control input and time
        % z = [time_1, w_m, time_2, w_m, ..., time_k, w_m];
        % ce_param.m = number of 'set of control input and time'
        % ce_param.dim = size of 'a set of control input and time'
        ce_param.ini_v  = R.u_linear;
        ce_param.m      = R.m_ce;
        ce_param.dim    = R.dim_ce;
    case 'unicycle_full'
        % trajectory is parameterized by set of control input and time
        % z = [time_1, v_1, w_1, ..., time_m, v_m, w_m];
        % ce_param.m = number of 'set of control input and time'
        % ce_param.dim = size of 'a set of control input and time'
        ce_param.m      = R.m_ce;
        ce_param.dim    = R.dim_ce;
    otherwise
        fprintf('ce_param are not defined\r\n');
end

%% ce initial setup
s = ce_initial_setup(sp,gp,R,ce_param,sinit);     % update s.mu, s.sigma, s.weight

% when sp == gp
if(isequal(sp,gp))
    path_final  = sp;
    input_final = [];
    z_final     = [];
    cost_final  = 0;
    flag        = 0;
    return;
end

%% start iteration
n_iter = 0;
while(n_iter <= N_iter)
    n_iter = n_iter + 1;
    
    cost.terminal       = zeros(1,traj_num);                % terminal cost
    cost.traj           = zeros(1,traj_num);                % running cost
    
    sampled_trajectory  = cell(1,traj_num);
    sampled_control     = cell(1,traj_num);
    
    Z = zeros(traj_num, ce_param.dim * ce_param.m);         % a finite-dimensional parameterizaiton of trajectories
    
    n_traj = 0;
    iter_loop = 0;
    while(n_traj < traj_num)
        n_traj = n_traj + 1;
        iter_loop = iter_loop + 1;
        
        % making path further is impossible
        is_true=(iter_loop>100)&(n_traj==1);
        if(is_true==1)
            flag = -1;
            break;
        end
        
        % sampling and generate trajectory using sampled outputs
        switch R.dynamic
            case 'pointmass'
                if use_mex
                    z = ce_state_sampling_mex(s.mu,s.sigma,ce_param.dim,ce_param.m,R.x_range);
                else
                    z = ce_state_sampling(s.mu,s.sigma,ce_param.dim,ce_param.m,R.x_range);
                end
                [path_gen,ctrl_gen] = ce_generate_point_path(sp(1:2),gp(1:2),z,ce_param.m,R.dist_interval,R,use_mex);
                Z(n_traj,:) = z;
            case {'unicycle_part','unicycle'}
                if use_mex
                    z = ce_input_sampling_mex(s.mu,s.sigma,ce_param.dim,ce_param.m,R.u_range);
                    [path_gen,ctrl_gen,outside] = ...
                    ce_generate_trajectory_mex(sp,z,ce_param.dim,ce_param.m,R.x_range,R.u_linear,R.dt);
                else
                    z = ce_input_sampling(s.mu,s.sigma,ce_param.dim,ce_param.m,R.u_range);
                    [path_gen,ctrl_gen,outside] = ce_generate_trajectory(sp,z,ce_param.dim,ce_param.m,R.x_range,R.u_linear,R.dt);
                end
                                                
                if(outside==1)
                    n_traj = n_traj - 1;
                    continue;
                end
                Z(n_traj,:) = z;
        end
        
        % check feasibility (trajectory)
        is_unsafe = checkFeasibility(path_gen,param.interval_collision,R,map,use_mex);
        if(is_unsafe)
            n_traj = n_traj - 1;
            continue;
        end
        
        % check feasibility (control)
        is_unsafe = checkControl(ctrl_gen,R);
        if(is_unsafe)
            n_traj = n_traj - 1;
            continue;
        end
        
        % compute cost of trajectory
        [cost_terminal,cost_traj] = ce_compute_cost(path_gen,gp,R,param,map);
        
        sampled_trajectory{1,n_traj}    = path_gen;
        sampled_control{1,n_traj}       = ctrl_gen;
        cost.terminal(n_traj)           = cost_terminal;
        cost.traj(n_traj)               = cost_traj;
    end % while(n_traj < traj_num)
        
    if(flag == -1 && n_traj < max(N_el1,N_el2))
        path_final = []; input_final = []; z_final = []; cost_final = 0;
        flag = -1;
        return;
    else
        sampled_trajectory  = sampled_trajectory(1,1:n_traj);
        sampled_control     = sampled_control(1,1:n_traj);
        cost.terminal       = cost.terminal(1,1:n_traj);
        cost.traj           = cost.traj(1,1:n_traj);
    end
    
    % find the elite set
    sorted_idx  = ce_find_elite_index(cost,N_el1,N_el2);   
    Z_elite     = Z(sorted_idx,:);
    
    % update s.mu, s.sigma
    s = ce_param_update(s,ce_param,Z_elite,n_iter);
    
    % among elite traj, find the low-cost traj 
    path_final  = sampled_trajectory{sorted_idx(1)};
    input_final = sampled_control{sorted_idx(1)};
    z_final     = Z(sorted_idx(1),:);               % selected Elite state
    cost_final  = cost.traj(sorted_idx(1));
    
    % break condition
    if(norm(s.sigma,2)<0.01)
        break;
    end

end % while (n_iter < N_iter)

end
