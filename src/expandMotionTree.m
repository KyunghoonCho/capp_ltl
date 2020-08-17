%% THE LOW-LEVEL SAMPLING-BASED EXPLORATION ALGORITHM
%   Extends sampling tree.
function [T,idx_acc,flag] = expandMotionTree(DP,R,T,z_sel,d_sel,g_target,highlevel_plan,idx_acc,map)

global param history use_mex

do_rewiring = 1;

% plot setting
plot_every_iteration = true;
plot_3d = true;
plot_detail = true;
save_every_iteration = true;

% probability of selecting random vertexes
probsv_rand = param.probsv_rand;

% threshold distance to apply long-extension
planning_threshold = 1*param.planning_threshold;

% output-flag
flag = 0;  % 0: normal, 1: error

%% start extention
iter = 0;
while iter < param.texplore
    iter = iter + 1;
    
    % a high-level state to be reached
    idx_HS      = g_target;
    
    % select a target vertex
    v_target    = selectTargetVertex(DP.D,idx_HS,R);
    
    % select a vertex to extend
    index_selected = selectVertex2Extend(T,R,z_sel,d_sel,v_target,probsv_rand);
    T.v.nsel(1,index_selected) = T.v.nsel(1,index_selected)+1;
    
    % start vertex
    x0 = T.v.x(:,index_selected);
    z0 = T.v.alpha(1,index_selected);
    
    %% find path-segment
    % check wheter to apply long-extention
%     is_long = ((R.dist_fun_array_ws(v_target(1:2),x0(1:2)) >= planning_threshold) && (idx_HS>=DP.index_RI)) ...
%         &&(param.allow_longExtension == 1);
    is_long = (R.dist_fun_array_ws(v_target(1:2),x0(1:2)) >= planning_threshold) ...
        && (param.allow_longExtension == 1);
    if ~is_long
        fprintf('(normal) ');
        [path_gen,ctrl_gen] = extendTree(x0,v_target,R);
        history.cnt_normal  = history.cnt_normal + 1;
        
        N_segment = 1;
        idx_p_segment = [1 size(path_gen,2)];
        idx_c_segment = [1 size(ctrl_gen,2)];
    else
        sinit = R.init_cov;
        [path_gen,ctrl_gen,idx_p_segment,idx_c_segment,~,~,is_failed] ...
            = runCE(x0,v_target,R,param,map,sinit,use_mex);
        history.cnt_long = history.cnt_long + 1;
        
        if(is_failed)
            fprintf('(long-failed) ');
            flag = 1;
            return;
        else
            N_segment = size(idx_p_segment,1);
            fprintf('(long) ');
        end
    end
    
    if size(path_gen,2) <= 1
        fprintf('path-length is too short: %d \n',size(path_gen,2));
        flag = 1;
        return;
    end
    
    x_cur       = x0;
    z_cur       = z0;
    idx_nearest = index_selected;
    
    traj_seg    = zeros(R.dim_x,size(path_gen,2));
    idx_ts      = 1;
    traj_seg(:,idx_ts) = x_cur;
    
    %% add path-segment to tree
    for nidx_seg = 1:1:N_segment
        pseg_cur    = path_gen(:,idx_p_segment(nidx_seg,1):idx_p_segment(nidx_seg,2));
        useg_cur    = ctrl_gen(:,idx_c_segment(nidx_seg,1):idx_c_segment(nidx_seg,2));
        x_next      = pseg_cur(:,end);
        
        % chceck feasibility 1
        is_safe_neigh = 1;
        is_unfeas_neigh_t = checkFeasibility(pseg_cur,param.interval_collision,R,map,use_mex);
        
        if is_unfeas_neigh_t
            is_safe_neigh = 0;
        else
            [is_failed,z_new] = checkAutomaton(pseg_cur,z_cur,DP);
            if is_failed; is_safe_neigh = 0; end
        end
        if is_safe_neigh == 0
            fprintf('not safe ');
            if iter == 1; flag = 1; end
            return;
        end
        
        % compute cost
        cost_pseg_neigh_new = ...
            calculateCost(pseg_cur,map,param.interval_cost,param.cost_type,R,1,use_mex);
        cost_neigh_new = T.v.cost(1,idx_nearest) + cost_pseg_neigh_new;
        
        idx_parent_min  = idx_nearest;
        z_next          = z_new;
        c_min           = cost_neigh_new;
        pseg_cost       = cost_pseg_neigh_new;
        path_cur        = [T.v.path{1,idx_nearest} x_next];
        path_cost       = cost_neigh_new;
        
        %% connect along a minimum-cost path
        if do_rewiring
            idx_found_temp = find(T.v.alpha(1:T.num_node) == z_cur);
            num_nodes_temp = length(idx_found_temp);
            dist_temp = min(param.dist_near,param.gamma_near*(log(num_nodes_temp)/num_nodes_temp)^(1/R.dim_ws));
            idx_near = findNear(T,x_next,z_cur,R.idx_ws,dist_temp);
            if ~isempty(idx_near)
                for nidx_near = 1:1:size(idx_near,2)
                    idx_near_sel = idx_near(1,nidx_near);
                    x_neigh = T.v.x(:,idx_near_sel);
                    z_neigh = T.v.alpha(1,idx_near_sel);
                    
                    if(idx_near_sel == idx_nearest || isequal(x_next,x_neigh))
                        continue;
                    end
                    
                    if use_mex
                        [pseg_neigh_new, useg_neigh_new] = extend_rewire_mex_mex(x_neigh,x_next,R.num_dyn,R.dim_x,R.dim_u,R.u_range,R.dt,R.rho);
                    else
                        [pseg_neigh_new, useg_neigh_new] = extendRewire(x_neigh,x_next,R);
                    end
                    
                    if(isempty(pseg_neigh_new)); continue; end
                    traj_seg_tmp = [x_neigh(1:R.dim_ws,:) x_next(1:R.dim_ws,:)];
                    
                    % check feasibility 2
                    is_safe_neigh = 1;
                    is_unfeas_neigh_t = checkFeasibility(pseg_neigh_new,param.interval_collision,R,map,use_mex);
                    % is_unfeas_neigh_c = checkControl(useg_neigh_new,R);
                    is_unfeas_neigh_c = 0;
                    
                    if (isempty(pseg_neigh_new)||is_unfeas_neigh_t || is_unfeas_neigh_c)
                        is_safe_neigh = 0;
                    else
                        [is_failed,z_neigh_new] = checkAutomaton(traj_seg_tmp,z_neigh,DP);
                        if is_failed; is_safe_neigh = 0; end
                    end
                    
                    if is_safe_neigh==1
                        cost_pseg_neigh_new = calculateCost(pseg_neigh_new,map,param.interval_cost,param.cost_type,R,1,use_mex);
                        cost_neigh_new = T.v.cost(1,idx_near_sel) + cost_pseg_neigh_new;
                        
                        is_true = (cost_neigh_new < c_min);
                        if is_true
                            c_min           = cost_neigh_new;
                            idx_parent_min  = idx_near_sel;
                            z_next          = z_neigh_new;
                            pseg_cur        = pseg_neigh_new;
                            useg_cur        = useg_neigh_new;
                            pseg_cost       = cost_pseg_neigh_new;
                            path_cur        = [T.v.path{1,idx_parent_min} pseg_neigh_new(:,2:end)];
                            path_cost       = cost_neigh_new;
                        end
                    end
                end
            end
        end % end rewiring
        
        x_cur = x_next;
        z_cur = z_next;
        d_cur = findIndex(DP.D,DP.index_RI,x_next);
        g_cur = DP.num_Z*(d_cur-1) + z_cur;
        if g_cur <= 0
            fprintf('error: g_cur:%d, d_cur:%d, z_cur:%d ',g_cur,d_cur,z_cur);
            if iter == 1; flag = 1; end
            return;
        end
        
        idx_ts = idx_ts+1;
        traj_seg(:,idx_ts) = x_cur;
        
        %% rewire the tree
        T.num_node = T.num_node+1;
        if T.num_node <= T.MAXSIZE
            T.v.x(:,T.num_node)         = x_cur;
            T.v.d(:,T.num_node)         = d_cur;
            T.v.alpha(:,T.num_node)     = z_cur;
            T.v.nsel(:,T.num_node)      = 0;
            T.e.parent(:,T.num_node)    = idx_parent_min;
            T.e.rewire(:,T.num_node)    = 0;
            T.e.child{1,idx_parent_min} = [T.e.child{1,idx_parent_min} T.num_node];
            T.e.u{1,T.num_node}         = useg_cur;
            
            T.v.path{1,T.num_node}      = path_cur;
            T.v.pseg{1,T.num_node}      = pseg_cur;
            T.v.pseg_cost(1,T.num_node) = pseg_cost;
            T.v.cost(1,T.num_node)      = path_cost;
            T.v.cost_p(1,T.num_node)    = calculateCostP(x_cur,map,param.cost_type);
            
            if param.do_compute_coverage
                % update cov
                x_grid = map.X_grid(1,:);
                y_grid = map.Y_grid(:,1)';
                dist_x = abs(x_grid - repmat(x_cur(1),1,size(x_grid,2)));
                dist_y = abs(y_grid - repmat(x_cur(2),1,size(y_grid,2)));
                [~,idx_minx] = min(dist_x);
                [~,idx_miny] = min(dist_y);
                if param_H.find_grid{1,z_cur}(idx_miny,idx_minx)==0
                    param_H.find_grid{1,z_cur}(idx_miny,idx_minx) = 1;
                    param_H.cov(1,g_cur) = param_H.cov(1,g_cur) + 1/param_H.sum_cov_d(1,d_cur);
                end
            end
            
            if do_rewiring
                idx_found_temp = find(T.v.alpha(1:T.num_node) == z_cur);
                num_nodes_temp = length(idx_found_temp);
                dist_temp = min(param.dist_near,param.gamma_near*(log(num_nodes_temp)/num_nodes_temp)^(1/R.dim_ws));
                % dist_temp = min(param.dist_near,param.gamma_near*(log(T.num_node)/T.num_node)^(1/R.dim_ws));
                idx_near = findNear(T,x_cur,[],R.idx_ws,dist_temp);
                if ~isempty(idx_near)
                    for nidx_near = 1:1:size(idx_near,2)
                        idx_near_sel = idx_near(1,nidx_near);
                        % skip already checked edges
                        is_true = (idx_near_sel == T.num_node) || (idx_near_sel == idx_parent_min);
                        if is_true; continue; end
                        
                        x_neigh         = T.v.x(:,idx_near_sel);
                        z_neigh         = T.v.alpha(1,idx_near_sel);
                        cost_neigh      = T.v.cost(1,idx_near_sel);
                        traj_seg_tmp    = [x_cur x_neigh];
                        
                        % check feasibility 3
                        is_safe_neigh = 1;
                        if use_mex
                            [pseg_neigh_new, useg_neigh_new] = extend_rewire_mex_mex(x_cur,x_neigh,R.num_dyn,R.dim_x,R.dim_u,R.u_range,R.dt,R.rho);
                        else
                            [pseg_neigh_new,useg_neigh_new] = extendRewire(x_cur,x_neigh,R);
                        end
                        is_unfeas_neigh_t = checkFeasibility(pseg_neigh_new,param.interval_collision,R,map,use_mex);
                        % is_unfeas_neigh_c = checkControl(useg_neigh_new,R);
                        is_unfeas_neigh_c = 0;
                        
                        if (isempty(pseg_neigh_new) || is_unfeas_neigh_t || is_unfeas_neigh_c)
                            is_safe_neigh = 0;
                        else
                            [is_failed,z_neigh_new] = checkAutomaton(traj_seg_tmp,z_cur,DP);
                            if is_failed; is_safe_neigh = 0; end
                        end
                        
                        if is_safe_neigh
                            path_neigh_new = [path_cur pseg_neigh_new(:,2:end)];
                            
                            % compute cost
                            cost_pseg_neigh_new = calculateCost(pseg_neigh_new,map,param.interval_cost,param.cost_type,R,1,use_mex);
                            cost_neigh_new      = T.v.cost(1,T.num_node) + cost_pseg_neigh_new;
                            
                            is_true = (cost_neigh_new < cost_neigh) && (z_neigh == z_neigh_new);
                            
                            % do rewiring
                            if is_true
                                T.v.alpha(:,idx_near_sel)       = z_neigh_new;
                                T.v.path{1,idx_near_sel}        = path_neigh_new;
                                T.v.pseg{1,idx_near_sel}        = pseg_neigh_new;
                                T.v.pseg_cost(1,idx_near_sel)   = cost_pseg_neigh_new;
                                T.v.cost(1,idx_near_sel)        = cost_neigh_new;
                                T.e.u{1,idx_near_sel}           = useg_neigh_new;
                                
                                idx_prev_parent_neigh = T.e.parent(1,idx_near_sel);
                                T.e.child{1,idx_prev_parent_neigh} = deleteChild(T.e.child{1,idx_prev_parent_neigh},idx_near_sel);
                                T.e.parent(1,idx_near_sel)      = T.num_node;
                                T.e.child{1,T.num_node}         = [T.e.child{1,T.num_node} idx_near_sel];
                                T.e.rewire(:,idx_near_sel)      = 1;
                                
                                % update child nodes
                                T = updateChild(T,idx_near_sel);
                            end
                        end
                    end
                end
            end % end rewiring
        else
            fprintf('reach the maximum size of tree ');
        end
        idx_nearest = T.num_node;
        
        % check solution
        if ismember(z_cur,DP.nbaParse.accepting_state_ids)
            idx_acc = [idx_acc T.num_node]; %#ok<AGROW>
        end    
    end
    
    %% plot
    %     if(param.iter <= param.num_nodes_plot_init)
    %         is_plot = param.do_plot == 1 && plot_every_iteration  == 1;
    %     else
    %         is_plot = param.do_plot == 1 && plot_every_iteration  == 1 && (T.num_node>=param.num_nodes_plot);
    %         if(is_plot)
    %             param.num_nodes_plot = param.num_nodes_plot + 30;
    %         end
    %     end
    if(param.do_plot == 1 && plot_every_iteration)
        traj_seg = traj_seg(:,1:idx_ts);
        if(size(traj_seg,2)>0)
            traj_ws = traj_seg(R.idx_ws,:);
        end
        
        % update pic_num
        if save_every_iteration ==1; param.pic_num = param.pic_num + 1; end
        
        % plot 3d
        if plot_3d
            hFig3d = plotPathWTree3D(DP,T,R.idx_ws,map,history.PATH_FOUND,history.AUTO_FOUND,d_sel,z_sel,v_target,...
                [],0,1,param.cost_type,highlevel_plan,g_target,is_long,traj_ws,param.fig_black);
            drawnow;
            pause(0.1);
            fig_pos = hFig3d.PaperPosition;
            hFig3d.PaperSize = [fig_pos(3) fig_pos(4)];
            
            % save 3d
            filename = sprintf("%s/pic/%d_3d_%d.png",history.dirname2save,param.fig_black,param.pic_num);
            if param.save_pic_mode
                print(hFig3d,filename,'-dpng');
            else
                saveas(hFig3d,filename);
            end
            pause(0.1);
            close(hFig3d)  % close 3d
        end
        
        % plot 2d
        if plot_detail
            hFig2d = plotTreeHs(DP,T,2,R.idx_ws,z_sel,d_sel,x0,map,param.cost_type,highlevel_plan,g_target,param.fig_black);
            if(size(traj_seg,2)>0 && is_long)
                plot(traj_ws(1,:),traj_ws(2,:),'o','MarkerSize',5,'LineWidth',1,'Color',rgb('OrangeRed'));
            end
            if(~isempty(v_target))
                plot(v_target(1),v_target(2),'rx','MarkerSize',8,'linewidth',1);
            end
            if(~isempty(history.PATH_FOUND))
                plot(history.PATH_FOUND(1,:),history.PATH_FOUND(2,:),'-','linewidth',2.75,'color',rgb('Yellow'));
                plot(history.PATH_FOUND(1,:),history.PATH_FOUND(2,:),'-','linewidth',2.0,'color',rgb('OrangeRed'));
            end
        else
            hFig2d = plotPathWTree(DP,T,R.idx_ws,map,history.PATH_FOUND,[],0,2,param.cost_type,param.fig_black);
        end
        drawnow;
        pause(0.1);
        fig_pos = hFig2d.PaperPosition;
        hFig2d.PaperSize = [fig_pos(3) fig_pos(4)];
        
        % save 2d
        if save_every_iteration
            filename = sprintf("%s/pic/%d_2d_%d.png",history.dirname2save,param.fig_black,param.pic_num);
            if param.save_pic_mode
                print(hFig2d,filename,'-dpng');
            else
                saveas(hFig2d,filename);
            end
        end
        pause(0.1);
        close(hFig2d)  % close 2d
    end
end

end
