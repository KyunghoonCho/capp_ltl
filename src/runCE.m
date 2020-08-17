%% APPLY CROSS-ENTROPY RANDOMIZED MOTION PLANNING
%   Input
%       sp: start point
%       gp: goal point
%       param: parameters
%       map: map-info
%       sinit: initial mu & sigma
%       use_mex: whether to use mex-function
function [path_out,input_out,idx_p_segment,idx_c_segment,z_out,cost_out,is_failed] ...
    = runCE(sp,gp,R,param,map,sinit,use_mex)

N_iter      = param.N_iter;         % number of iteration 50
traj_num    = param.traj_num;       % number of trajectory 50

switch param.cost_type
    case 'distance'
        cost_type = 1;
    case 'costmap'
        cost_type = 2;
end

if use_mex
    switch R.dynamic
        case 'pointmass'
            [path_final,input_final,z_final,cost_final,flag] = CE_MP_4mex_mex(sp,gp,...
                1,R.dim_ws,R.idx_ws,R.dist_weight,R.dim_x,R.dim_u,R.x_range,R.u_range,R.dt,R.num_elite_position,R.dist_interval,1,1,1, ...
                map.X,map.Y,map.C,map.C_unfeas,...
                cost_type,param.interval_cost,param.interval_collision,sinit,N_iter,traj_num);
        case 'unicycle_part'
            [path_final,input_final,z_final,cost_final,flag] = CE_MP_4mex_mex(sp,gp,...
                2,R.dim_ws,R.idx_ws,R.dist_weight,R.dim_x,R.dim_u,R.x_range,R.u_range,R.dt,1,1,R.u_linear,R.m_ce,R.dim_ce, ...
                map.X,map.Y,map.C,map.C_unfeas,...
                cost_type,param.interval_cost,param.interval_collision,sinit,N_iter,traj_num);
    end
else
    [path_final,input_final,z_final,cost_final,flag] = CE_MP(sp,gp,R,param,map,sinit,false);
end


if(flag==-1 || size(path_final,2) == 1)
    path_out    = [];
    input_out   = [];
    z_out       = [];
    cost_out    = 0;
    is_failed   = 1;
    
    idx_p_segment = [];
    idx_c_segment = [];
else
    path_out    = path_final;
    input_out   = input_final;
    z_out       = z_final;
    cost_out    = cost_final;
    is_failed   = 0;
    
    if use_mex
        [idx_p_segment,idx_c_segment] = decompose_trajectory_mex(path_out,R.idx_ws,R.segment_distance);
    else
        [idx_p_segment,idx_c_segment] = decompose_trajectory(path_out,R.idx_ws,R.segment_distance);
    end
end

end
