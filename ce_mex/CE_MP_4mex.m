% Run Cross-Entropy Randomized Motion Planning (mex-version).
function [path_final,input_final,z_final,cost_final,flag] = CE_MP_4mex(sp,gp,...
    R_dynamic,R_dim_ws,R_idx_ws,R_dist_w,R_dim_x,R_dim_u,R_xrange,R_urange,R_dt,R_num_e_p,R_dist_interval,R_ulinear,R_m_ce,R_dim_ce, ...
    map_X,map_Y,map_C,map_C_unfeas,...
    cost_type,interval_cost,interval_collision,sinit_sigma,N_iter,traj_num)

flag = 1;

%% set parameters
N_el1   = round(traj_num * 0.2);
N_el2   = round(traj_num * 0.1);

% parameters related with update
ce_param_alpha  = .9;
ce_param_beta   = .7;
ce_param_b_q    = 15;

switch R_dynamic
    case 1 % 'pointmass'
        % state sampling
        ce_param_m      = R_num_e_p;
        ce_param_dim    = 2 * 1;
    case 2 % 'unicycle_part'
        % trajectory is parameterized by set of control input and time
        % z = [time_1, w_m, time_2, w_m, ..., time_k, w_m];
        % ce_param_m = number of 'set of control input and time'
        % ce_param_dim = size of 'a set of control input and time'
        ce_param_m      = R_m_ce;
        ce_param_dim    = R_dim_ce;
    case 3 % 'unicycle_full'
        % trajectory is parameterized by set of control input and time
        % z = [time_1, v_1, w_1, ..., time_m, v_m, w_m];
        % ce_param_m = number of 'set of control input and time'
        % ce_param_dim = size of 'a set of control input and time'
        ce_param_m      = R_m_ce;
        ce_param_dim    = R_dim_ce;
    otherwise
        fprintf('ce_param are not defined\r\n');
        ce_param_m      = R_m_ce;
        ce_param_dim    = R_dim_ce;
end

%% ce initial setup
% update s_mu, s_sigma
[s_mu,s_sigma] = ce_initial_setup_mex(sp,gp,R_dynamic,R_ulinear,...
    ce_param_m,ce_param_dim,sinit_sigma);

path_final  = [];
input_final = [];
z_final     = [];
cost_final  = 0;

%% start iteration
n_iter = 0;
while(n_iter <= N_iter)
    n_iter = n_iter + 1;
    
    cost_terminal       = zeros(1,traj_num);                % terminal cost
    cost_traj           = zeros(1,traj_num);                % running cost
    
    sampled_trajectory  = zeros(R_dim_x*traj_num,5000);
    sampled_control     = zeros((R_dim_u+1)*traj_num,5000);
    end_idx_array       = zeros(2,5000);
    
    Z = zeros(traj_num, ce_param_dim * ce_param_m);         % a finite-dimensional parameterizaiton of trajectories
    
    n_traj      = 0;
    iter_loop   = 0;
    while(n_traj < traj_num)
        n_traj      = n_traj + 1;
        iter_loop   = iter_loop + 1;
        
        % making path further is impossible
        is_true=(iter_loop>100)&(n_traj==1);
        if(is_true==1)
            flag = -1;
            break;
        end
        
        % sampling and generate trajectory using sampled outputs
        switch R_dynamic
            case 1
                z                   = ce_state_sampling(s_mu,s_sigma,ce_param_dim,ce_param_m,R_xrange);
                [path_gen,ctrl_gen] = ce_generate_point_path(sp(1:2),gp(1:2),z,ce_param_m,R_dim_x,R_dim_u,R_urange,R_dist_interval);
                if(size(path_gen,2) ~= size(ctrl_gen,2)+1)
                    
                end
                
                Z(n_traj,:)         = z;
            case {2,3}
                z                   = ce_input_sampling(s_mu,s_sigma,ce_param_dim,ce_param_m,R_urange);
                
                [path_gen,ctrl_gen,outside] = ...
                    ce_generate_trajectory(sp,z,ce_param_dim,ce_param_m,R_xrange,R_ulinear,R_dt);
                
                if(outside==1)
                    n_traj = n_traj - 1;
                    continue;
                end
                Z(n_traj,:) = z;
            otherwise
                path_gen = [];
                ctrl_gen = [];
        end
        
        % check feasibility (trajectory)
        is_unsafe = ce_check_feasibility_mex(path_gen,interval_collision,...
            R_idx_ws,R_dim_ws,R_dim_x,R_xrange,map_X,map_Y,map_C_unfeas);
        if(is_unsafe)
            n_traj = n_traj - 1;
            continue;
        end
        
        % check feasibility (control)
        is_unsafe = ce_check_control_mex(ctrl_gen,R_dim_u,R_urange);
        if(is_unsafe)
            n_traj = n_traj - 1;
            continue;
        end
        
        % compute cost of trajectory
        [cost_terminal_tmp,cost_traj_tmp] = ce_compute_cost_mex(path_gen,gp,...
            R_dist_w,R_idx_ws,R_dim_ws,map_X,map_Y,map_C,cost_type,interval_cost);
        
        idx_x_update    = ((R_dim_x)*(n_traj-1)+1):1:(R_dim_x*n_traj);
        idx_u_update    = ((R_dim_u+1)*(n_traj-1)+1):1:((R_dim_u+1)*n_traj);
        sampled_trajectory(idx_x_update,1:size(path_gen,2))     = path_gen;
        sampled_control(idx_u_update,1:size(ctrl_gen,2))        = ctrl_gen;
        end_idx_array(:,n_traj) = [size(path_gen,2); size(ctrl_gen,2)];
        
        cost_terminal(1,n_traj)             = cost_terminal_tmp;
        cost_traj(1,n_traj)                 = cost_traj_tmp;
    end % while(n_traj < traj_num)
    
    if(flag == -1 && n_traj < max(N_el1,N_el2))
        path_final = []; input_final = []; z_final = []; cost_final = 0;
        flag = -1;
        return;
    else
        end_idx_array       = end_idx_array(:,1:n_traj);
        cost_terminal       = cost_terminal(1,1:n_traj);
        cost_traj           = cost_traj(1,1:n_traj);
        
        % find the elite set
        sorted_idx  = ce_find_elite_index_mex(cost_terminal,cost_traj,N_el1,N_el2);
        Z_elite     = Z(sorted_idx,:);
        
        % update s.mu, s.sigma
        [s_mu,s_sigma] = ce_param_update_mex(s_mu,s_sigma,...
            ce_param_m,ce_param_dim,ce_param_alpha,ce_param_beta,ce_param_b_q,Z_elite,n_iter);
        
        % among elite traj, find the low-cost traj
        idx_x_update    = ((R_dim_x)*(sorted_idx(1)-1)+1):1:(R_dim_x*sorted_idx(1));
        idx_u_update    = ((R_dim_u+1)*(sorted_idx(1)-1)+1):1:((R_dim_u+1)*sorted_idx(1));
        path_final      = sampled_trajectory(idx_x_update,1:end_idx_array(1,sorted_idx(1)));
        input_final     = sampled_control(idx_u_update,1:end_idx_array(2,sorted_idx(1)));
        
        z_final         = Z(sorted_idx(1),:);               % selected Elite state
        cost_final      = cost_traj(sorted_idx(1));
        
        % break condition
        %     if(norm(s_sigma,2)<0.01)
        %         break;
        %     end
    end
    
end % while (n_iter < N_iter)

end
