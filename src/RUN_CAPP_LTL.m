%% READY TO RUN CAPP-LTL
%   Establish an environment to run CAPP-LTL.
function RUN_CAPP_LTL(rnd_seed,scenario_num,iterMax,texplore,allow_longExtension,dynamic_type,...
        triangle_type,cost_type,LTLSPEC,AP_LTLSPEC,x_init,workspacefile,costmapfile,...
        do_plot,load_DP,do_compute_coverage,fig_black,save_pic_mode)

clc;
clearvars -except path_to_ltl2ba path_to_triangulate path_to_triangle use_mex ...
    rnd_seed scenario_num iterMax texplore allow_longExtension dynamic_type ...
    triangle_type cost_type LTLSPEC AP_LTLSPEC x_init P workspacefile ...
    costmapfile do_plot load_DP do_compute_coverage fig_black save_pic_mode
close all;

global path_to_ltl2ba path_to_triangulate path_to_triangle
global param history

exc_path.path_to_ltl2ba         = path_to_ltl2ba;
exc_path.path_to_triangulate    = path_to_triangulate;
exc_path.path_to_triangle       = path_to_triangle;

plot_per_automaton = true;

rng(rnd_seed);

%% load workspace
workspacename_0 = split(workspacefile,'workspace_');
workspacename_1 = split(workspacename_0{2},'.');
workspacename = workspacename_1{1};

load(workspacefile);
map.xmin    = min(P{1,1}.points(:,1));
map.xmax    = max(P{1,1}.points(:,1));
map.ymin    = min(P{1,1}.points(:,2));
map.ymax    = max(P{1,1}.points(:,2));

% load costmap
if strcmp(cost_type,'costmap')
    if ~isempty(costmapfile)
        load(costmapfile);
        if exist('gvs','var')
            map.xmin    = min(gvs.X01(1,:));
            map.xmax    = max(gvs.X01(1,:));
            map.ymin    = min(gvs.X02(:,1));
            map.ymax    = max(gvs.X02(:,1));
            map.X       = gvs.X01;
            map.Y       = gvs.X02;
            map.C       = gvs.Z0;
        else
            map.X       = X;
            map.Y       = Y;
            map.C       = C;
        end
        
        % modify cost
        if contains(workspacename,'helsinki')
            map.C = max(max(map.C)) - map.C;
            map.C = 0.2*(exp(map.C / 8)) + 1e-2;
        elseif contains(workspacename,'tree_exp')
            map.C = map.C - min(min(map.C));
            map.C = 4.0*(exp(map.C / 4.0));
            map.C = map.C - min(min(map.C)) + 5e-2;
        else
            map.C = map.C - min(min(map.C));
            map.C = 2 * map.C;
        end
    end
else
    x_range = map.xmin:0.2:map.xmax;
    y_range = map.ymin:0.2:map.ymax;
    [map.X,map.Y]   = meshgrid(x_range,y_range);
    map.C = zeros(size(map.X));
end

% update unfeasibility map
map.C_unfeas   = createObsMap(map,P);

%% set parameters
diameter = sqrt((map.xmax - map.xmin)^2+(map.ymax-map.ymin)^2);

% workspace dimension
R.dim_ws = 2;

% dynamic constraints
R.dynamic = dynamic_type;    % 'pointmass', 'unicycle_part'
switch R.dynamic
    case 'pointmass'
        R.num_dyn = 1;              % dynamic numbering (for mex)
        R.idx_ws = [1 2];           % index of workspace (x,y)
        R.dim_x = 2;                % dimension of robot state
        R.dim_u = 2;                % dimension of control state
        
        if size(x_init,1) > R.dim_x; x_init = x_init(1:R.dim_x,1); end
        
        % distance function
        dist_weight         = [1 1];
        R.dist_weight       = dist_weight;
        R.dist_fun          = @(x1,x2) sum(sqrt(dist_weight*((x1-x2).*(x1-x2))),2);
        R.dist_fun_ws       = @(x1,x2) sum(sqrt([1 1]*((x1-x2).*(x1-x2))),2);
        R.dist_fun_array    = @(x1,x2) sqrt(dist_weight*((x1-x2).*(x1-x2)));
        R.dist_fun_array_ws = @(x1,x2) sqrt([1 1]*( (x1-x2).*(x1-x2) ));
        
        % constraints
        vmin = -0.75;       vmax = +0.75;
        R.x_range   = [map.xmin map.xmax; map.ymin map.ymax];
        R.u_range   = [vmin vmax; vmin vmax];
        R.dt = 0.1;
        R.tN = 10;      % total extension step = param.dt * param.tN
        
        % Allow long extension
        if(allow_longExtension == 1)
            R.segment_distance = 1;
            
            R.num_elite_position = 6;
            R.dist_interval = 0.5;
            
            x_width = (map.xmax - map.xmin);
            y_width = (map.ymax - map.ymin);
            R.init_cov = [x_width^2/10 0 ; 0 y_width^2/10];
        end
    case 'unicycle_part'
        R.num_dyn = 2;              % dynamic numbering (for mex)
        R.idx_ws = [1 2];           % index of workspace (x,y)
        R.dim_x = 3;                % dimension of robot state
        R.dim_u = 2;                % dimension of control state
        
        % distance function
        dist_weight         = [1 1 0.2];
        R.dist_weight       = dist_weight;
        R.dist_fun          = @(x1,x2) sum(sqrt(dist_weight*( (x1-x2).*(x1-x2) )),2);
        R.dist_fun_ws       = @(x1,x2) sum(sqrt([1 1]*( (x1-x2).*(x1-x2) )),2);
        R.dist_fun_array    = @(x1,x2) sqrt(dist_weight*( (x1-x2).*(x1-x2) ));
        R.dist_fun_array_ws = @(x1,x2) sqrt([1 1]*( (x1-x2).*(x1-x2) ));
        
        % constraints
        u_linear = 1;
        wmin = -1.2*pi;   wmax = +1.2*pi;
        R.x_range   = [map.xmin map.xmax; map.ymin map.ymax; -pi +pi];
        R.u_linear  = u_linear;
        R.u_range   = [0.2 u_linear+0.1; wmin wmax];
        R.dt = 0.1;
        R.tN = 10;                  % total extension time = param.dt * param.tN
        
        % Rewiring
        R.rho = R.u_linear/wmax;
        
        % CE related
        if allow_longExtension == 1
            R.segment_distance = 0.2;
            
            R.m_ce = 6;
            R.dim_ce = 2;
            
            R.init_cov = 2*[1 0.1];
        end
end

% when 'rho' is not set
if ~isfield(R,{'rho'}); R.rho = 0; end

% Key planning parameters
%   probh_rand: probability of selecting random high-level plan.
%   probsv_rand: probability of selecting random vertexsave_pic_mode
%   probt_ri: probability of selecting region of high-level state.
%   planning_threshold: threshold distance between 'ce-method' and (normal) tree extension.
probh_rand = 0.4;
probsv_rand = 0.2;
planning_threshold = 0.45*diameter/sqrt(2);
if contains(workspacename,'helsinki')
    probt_ri = 0.7;
elseif contains(workspacename,'tree_exp')
    probt_ri = 0.3;
else
    probt_ri = 0.5;
end

% threshold for near distance
gamma_near      = 20;
dist_near       = 0.15*diameter/sqrt(2);

% margin for region of interest
RI_margin           = 0;

if(allow_longExtension)
    N_iter      = 300;  % number of iteration 50
    traj_num    = 300;  % number of trajectory 50
else
    N_iter      = 0;
    traj_num    = 0;
end

interval_collision  = 0.1;  % (distance) interval for checking collisoin
interval_cost       = 0.01; % (distance) interval for calculating cost of trajectory 
                            % (especially when (cost_type = 'costmap'))

%% set global parameter
param = initializeParam(do_plot,triangle_type,cost_type,...
    interval_collision,interval_cost,allow_longExtension,...
    probh_rand,probsv_rand,probt_ri,planning_threshold,...
    gamma_near,dist_near,RI_margin,N_iter,traj_num,iterMax,texplore,...
    do_compute_coverage,fig_black,save_pic_mode);

history.data = zeros(3,1000);
history.cnt = 0;
history.elapsed_time = 0;
history.cnt_extention = zeros(2,1000);
history.cnt_normal = 0;
history.cnt_long = 0;

history.dirname2save = ['./result/' workspacename '_sc' int2str(scenario_num) '_tr' int2str(param.triangle_type) ...
    '_' R.dynamic '_' int2str(param.allow_longExtension) '_' param.cost_type '/' int2str(rnd_seed)];
history.dirname2save_DP = ['./result/' workspacename '_sc' int2str(scenario_num) '_tr' int2str(param.triangle_type) ...
    '_' R.dynamic '_' int2str(param.allow_longExtension) '_' param.cost_type];
if(exist(history.dirname2save,'dir')==0)
    mkdir(history.dirname2save);
    mkdir([history.dirname2save '/pic']);
else
    if(exist([history.dirname2save '/pic'],'dir')==0)
        mkdir([history.dirname2save '/pic']);
    end
end
history.PATH_FOUND = [];
history.AUTO_FOUND = [];

%% run CAPP-LTL
[DP,T,traj_out,input_out,cost_out,is_solutionFound] = CAPP_LTL(x_init,LTLSPEC,AP_LTLSPEC,R,P,map,exc_path,load_DP);

% save result
filename_final = [history.dirname2save '/final_result.mat'];
save(filename_final,'DP','T','traj_out','input_out','cost_out','R','param');

filename_history = [history.dirname2save '/history.mat'];
save(filename_history,'history');

%% plot
if(~is_solutionFound)
    fprintf('failed in searching for a feasible solution\n');
else
    filename2 = sprintf("%s/%d_path_final.png",history.dirname2save,param.fig_black);
    hFig = plotPath(DP,R.idx_ws,map,traj_out,cost_out,1,param.cost_type,param.fig_black);
    % hFig = plotPathWTree(DP,T,R.idx_ws,map,traj_out,cost_out,[],1,param.cost_type);
    drawnow;
    pause(0.1);
    fig_pos = hFig.PaperPosition;
    hFig.PaperSize = [fig_pos(3) fig_pos(4)];
    if(param.save_pic_mode)
        print(hFig,filename2,'-dpng');
    else
        saveas(hFig,filename2);
    end
    pause(0.1);
    close(hFig)  % close
end

% save tree per automaton
if plot_per_automaton
    for nidx_z = 1:1:DP.num_Z
        filename3 = sprintf("%s/%d_tree_per_automaton_%d.png",history.dirname2save,param.fig_black,nidx_z);
        hFig3 = plotTreePa(DP,T,R.idx_ws,nidx_z+1,map,param.cost_type,nidx_z,param.fig_black);
        drawnow;
        pause(0.1);
        fig_pos = hFig3.PaperPosition;
        hFig3.PaperSize = [fig_pos(3) fig_pos(4)];
        if param.save_pic_mode
            print(hFig3,filename3,'-dpng');
        else
            saveas(hFig3,filename3);
        end
        pause(0.1);
        close(hFig3)  % close
    end
end
end
