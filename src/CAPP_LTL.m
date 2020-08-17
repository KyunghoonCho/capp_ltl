%% RUN CAPP-LTL
%   Cost Aware Path Planning with LTL constraints : CAPP_LTL
%       - Basic framework: multi-layered synergistic approach motion planning.
%           The algorithm is composed of 'high-level planning' and 'low-level planning'.
%               (1) high-level planning : Dijkstra¡¯s single-source shortest path algorithm.
%               (2) low-level planning  : Rapidly-exploring Random Tree
%               (RRT*) with long-extension (CE-MP).
%
%       - Reference:
%           Kyunghoon Cho, Junghun Suh, Clarie J. Tomlin, and Songhwai Oh,
%           "Cost-Aware Path Planning under Co-Safe Temporal Logic
%           Specifications," IEEE Robotics and Automation Letters, 
%           vol. 2, no. 4, pp. 2308-2315, Oct. 2017.

function [DP,T,traj_out,ctrl_out,cost_out,is_solutionFound] = CAPP_LTL(x_init,LTLSPEC,AP_LTLSPEC,R,P,map,exc_path,load_DP)

global param history param_H use_mex

% parameter computation mode
%   1 : (shortest distance from state z to the set of accepting states)
%   2 : (shortest distance from state (z,d) to the set of accepting states)
mode_parameter = 2;

%% 1: set discrete-planner
filenameDP = [history.dirname2save_DP '/DP.mat'];
if ~load_DP || ~exist(filenameDP,'file')
    % compute automaton
    nbaParse = parseNeverClaim(LTLSPEC, exc_path.path_to_ltl2ba);
    
    % compute decomposition
    DP = DiscretePlanner(P,nbaParse,AP_LTLSPEC,param.RI_margin,use_mex);
    if(param.triangle_type==0)
        DP = DP.naive_decomposition(P,x_init(1:2));
    else
        DP.triangulation(P,exc_path.path_to_triangulate,exc_path.path_to_triangle,param.triangle_type);
        % DP.Plot(1);
        
        % set transition
        DP = DP.make_transition();
    end
    
    % compute edge cost in discrete system
    DP = DP.compuete_edgeCost(R,map,param);
    
    % compute abstraction
    DP = DP.build_productSpace();
    DP = DP.compute_C_GRAPH();
    DP = DP.compute_parameters(mode_parameter);
    DP = DP.compute_prePath();
    
    save(filenameDP,'DP');
else
    % load existing discrete planner
    load(filenameDP); %#ok<LOAD>
end
 
% DP.Plot_ver2(1);
% hFig = figure(1);
% filename2save = ['tr' int2str(param.triangle_type) '.png'];
% print(hFig,filename2save, '-dpng', '-r300','-noui'); % 300 DPI

%% update param_H (parameters for high-level planning)
param_H.area_g = zeros(1,DP.num_G);
for nidx_d = 1:1:DP.num_D
    idx_dz = DP.num_Z*(nidx_d-1)*ones(1,DP.num_Z)+(1:DP.num_Z);
    param_H.area_g(1,idx_dz) = DP.area_D(1,nidx_d)*ones(1,size(idx_dz,2));
end
param_H.nrsel = zeros(1,DP.num_G);      % the number of times the high level state (z,d) has been selected in the past.                                       
param_H.cov = zeros(1,DP.num_G);        % estimates the progress made by sampling-based motion planner in the high-level state.
if param.do_compute_coverage
    param_H.sum_cov_d = zeros(1,DP.num_D);
    for nidx_rx = 1:1:size(map.X_grid,1)
        for nidx_cx = 1:1:size(map.X_grid,2)
            p_sel = [map.X_grid(nidx_rx,nidx_cx); map.Y_grid(nidx_rx,nidx_cx)];
            ind_sel = findIndex(DP.D,DP.index_RI,p_sel);
            if(ind_sel>0)
                param_H.sum_cov_d(1,ind_sel) = param_H.sum_cov_d(1,ind_sel) + 1;
            end
        end
    end
    param_H.fine_grid = cell(1,DP.num_Z);
    for nidx_z = 1:1:DP.num_Z
        param_H.find_grid{1,nidx_z} = zeros(size(map.X_grid));
    end
end

% find h_w
switch mode_parameter
    case 1
        h_w = zeros(1,DP.num_G);
        for nidx_z = 1:1:DP.num_Z
            idx_g = DP.num_Z*((1:DP.num_D) - 1) + nidx_z*ones(1,DP.num_D);
            h_w(1,idx_g) = DP.w_z(1,nidx_z)*ones(1,size(idx_g,2));
        end
    case 2
        h_w = DP.w_zd;
end

%% initialize search tree
MAXSIZE = 200000;        % the maximum number of nodes in the tree-structure
T       = initializeTree(x_init,DP.nbaParse.initial_state_ids,MAXSIZE,DP.D,DP.index_RI,map,param.cost_type);

%% search for a feasible solution
MINCOST = 1e7;      % minimum cost of found path
idx_acc = [];       % accepting tree index

tic;
param.iter = 0;
param.num_nodes_plot = 300;
param.num_nodes_plot_init = param.num_nodes_plot;
cnt_traj = 0;
while param.iter < param.iterMax
    %% update heuristic weights
    w_zd = updateHeuristicWeight(h_w,param_H.area_g,param_H.cov,param_H.nrsel,1,2.5,1);
    
    %% select initial state to extend
    [z_init,d_init,g_init] = selectInitialHS(T,w_zd,DP.num_Z,DP.nbaParse.accepting_state_ids);
    if g_init<=0; continue; end
    
    %% target state selection & discrete planning
    r_seed = rand;
    if r_seed <= param.probh_rand
        g_to            = (randi(DP.num_D)-1)*DP.num_Z + z_init;
        highlevel_plan  = DP.prePath_g{g_init,g_to};
        if isempty(highlevel_plan)
            fprintf('high-level plan not found\n');
            continue;
        end
    else
        z_to = DP.find_targetPoint_proposed(T,z_init,true);
        % z_to    = DP.nbaParse.accepting_state_ids(randi(length(DP.nbaParse.accepting_state_ids)));
        highlevel_plan = DP.prePath_z{g_init,z_to};
        if isempty(highlevel_plan)
            fprintf('high-level plan not found\n');
            continue;
        end
    end
    
    %% select target high-level state
    g_target = selectTargetHS(DP,highlevel_plan,param.probt_ri);
        
    %% extend the tree structure
    fprintf('[%d] ',param.iter);
    [T,idx_acc,flag] = expandMotionTree(DP,R,T,z_init,d_init,g_target,highlevel_plan,idx_acc,map);
    history.elapsed_time = toc;
    history.idx_acc = idx_acc;
    param_H.nrsel(1,g_init) = param_H.nrsel(1,g_init) + 1;
    fprintf('num_nodes=%d, time=%.2f, MINCOST=%.2f\n',T.num_node,history.elapsed_time,MINCOST);
        
    if size(idx_acc,2) > 0
        cost_traj = T.v.cost(1,idx_acc);
        [cost_sel,idx_min] = min(cost_traj);
        if cost_sel < MINCOST
            % traj_out = T.v.path{1,idx_acc(1,idx_min)};
            traj_out = recoverPath(T.v.x(:,1:T.num_node),T.e.parent(:,1:T.num_node),T.v.pseg(1,1:T.num_node),idx_acc(1,idx_min));
            auto_out = recoverPathAutomaton(T.v.alpha(1,1:T.num_node),T.e.parent(:,1:T.num_node),T.v.pseg(1,1:T.num_node),idx_acc(1,idx_min));
            ctrl_out = recoverControl(T.e.parent(:,1:T.num_node),T.e.u(1,1:T.num_node),idx_acc(1,idx_min));
            cost_out = T.v.cost(1,idx_acc(1,idx_min));
            
            MINCOST = cost_sel;
            time_history = toc;
            history.cnt = history.cnt + 1;
            history.data(:,history.cnt) = [MINCOST;param.iter;time_history];
            history.cnt_extention(:,history.cnt) = [history.cnt_normal; history.cnt_long];
            history.PATH_FOUND  = traj_out;
            history.AUTO_FOUND  = auto_out;
            
            %% plot
            if param.do_plot==1
                cnt_traj = cnt_traj + 1;
                fprintf('update solution (cost_sel:%.2f, time: %.2f)\n',cost_sel,time_history);
                hFig = plotPathWTree(DP,T,R.idx_ws,map,traj_out,cost_sel,time_history,1,param.cost_type,param.fig_black);
                drawnow;
                pause(0.1);
                % path_out = controlByInput(x_init, ctrl_out, R);
                % plot(path_out(1,:),path_out(2,:),'-.','linewidth',2,'color','c');
                fig_pos = hFig.PaperPosition;
                hFig.PaperSize = [fig_pos(3) fig_pos(4)];
                
                filename1 = sprintf("%s/%d_path_%d.png",history.dirname2save,param.fig_black,cnt_traj);
                if param.save_pic_mode
                    print(hFig,filename1,'-dpng')
                else
                    saveas(hFig,filename1);
                end
                
                pause(0.1);
                close(hFig)  % close
            end
        end
    end
    
    if flag == 0; param.iter = param.iter + 1; end
end


if ~isempty(idx_acc)
    is_solutionFound = 1;
else    % when solution is not found
    traj_out = [];
    ctrl_out = [];
    cost_out = 0;
    is_solutionFound = 0;
end

end
