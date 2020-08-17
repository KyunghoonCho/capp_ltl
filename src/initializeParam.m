% Initialize parameters.
function param = initializeParam(do_plot,triangle_type, ...
    cost_type,interval_collision,interval_cost,allow_longExtension,...
    probh_rand,probsv_rand,probt_ri,planning_threshold,gamma_near,dist_near,...
    RI_margin,N_iter,traj_num,iterMax,texplore,do_compute_coverage,fig_black, ...
    save_pic_mode)
param.do_plot = do_plot;
param.pic_num = 0;
param.triangle_type = triangle_type;
param.cost_type = cost_type;
param.interval_collision = interval_collision;
param.interval_cost = interval_cost;
param.allow_longExtension = allow_longExtension;

param.probh_rand = probh_rand;
param.probsv_rand = probsv_rand;
param.probt_ri = probt_ri;
param.planning_threshold = planning_threshold;

param.gamma_near    = gamma_near;
param.dist_near     = dist_near;

param.RI_margin = RI_margin;

param.N_iter    = N_iter;
param.traj_num  = traj_num;

param.iterMax = iterMax;
param.texplore = texplore;

param.do_compute_coverage = do_compute_coverage;

param.fig_black = fig_black;
param.save_pic_mode = save_pic_mode;
end
