function [cost_terminal,cost_traj] = ce_compute_cost(traj,gp,R,param,map)
% cost_terminal : distance between goal point between end of the trajectory
% cost_traj     : cost of trajectory 'distance' and 'environment map'

if(size(gp,1)==2)
    cost_terminal = norm(gp(1:2,1)-traj(1:2,end));
else
    cost_terminal = R.dist_fun(gp(:,1),traj(:,end));
end

switch param.cost_type
    case 'distance'
        cost_tmp = R.dist_fun(traj(:,2:end),traj(:,1:end-1));
        
    case 'costmap'
        choose_method = 3;
        
        interval_cost = max(0.2,param.interval_cost);
        
        % extend trajectory
        if(size(traj,2)==1)
            traj_extended = traj;
        else
            traj_extended = [];
            for nidx_traj = 2:1:size(traj,2)
                sp = traj(1:2,nidx_traj-1);
                gp = traj(1:2,nidx_traj);
                diff = gp-sp;
                dist = norm(diff);
                if(dist~=0)
                    num_points = floor(dist/interval_cost);
                    traj_seg = repmat(sp,1,num_points+1) + ...
                        repmat(0:num_points,2,1).*repmat((interval_cost/dist)*diff,1,num_points+1);
                else
                    traj_seg = sp;
                end
                
                traj_extended = [traj_extended traj_seg]; %#ok<AGROW>
            end
        end
        
        traj_extended = (unique(traj_extended','rows'))';
        
        switch choose_method
            case 1 % move each point to the grid point of map (for-loop)
                size_row = size(map.Y,1);
                size_col = size(map.X,2);
                index_grid = zeros(2,size(traj_extended,2));
                for nidx_t = 1:1:size(traj_extended,2)
                    xp_traj = traj_extended(:,nidx_t);
                    x = map.X(1,:);
                    y = map.Y(:,1)';
                    dist_x = abs(repmat(xp_traj(1),1,size_col) - x);
                    dist_y = abs(repmat(xp_traj(2),1,size_row) - y);
                    [~,ind_X] = min(dist_x);
                    [~,ind_Y] = min(dist_y);
                    index_grid(:,nidx_t) = [ind_Y; ind_X];
                end
                
                index_grid_unique = (unique(index_grid','rows'))';
                
                idx_z = sub2ind(size(map.C),index_grid_unique(1,:),index_grid_unique(2,:));
                z_traj = map.C(idx_z);
                cost_tmp = sum(z_traj,2);
                
            case 2 % move each point to the grid point of map (round)
                traj_x = traj_extended(1,:);
                traj_y = traj_extended(2,:);
                                               
                xmin = min(map.X(1,:));
                ymin = min(map.Y(:,1));
                xgrid = map.X(1,2) - map.X(1,1);
                ygrid = map.Y(2,1) - map.Y(1,1);
                
                round_num = [(1/xgrid); (1/ygrid)];
                                
                traj_rounded = [round(traj_x*round_num(1))/round_num(1);  round(traj_y*round_num(2))/round_num(2)];
                
                idx_trajx = (traj_rounded(1,:)-repmat(xmin,1,size(traj_rounded,2)))/xgrid + 1;
                idx_trajy = (traj_rounded(2,:)-repmat(ymin,1,size(traj_rounded,2)))/ygrid + 1;
                idx_trajx = round(idx_trajx);
                idx_trajy = round(idx_trajy);
                
                [~,idx_nv] = find(idx_trajy==0);
                if(~isempty(idx_nv))
                end
                idx_z = sub2ind(size(map.C),idx_trajy,idx_trajx);
                z_traj = map.C(idx_z);
         
                cost_tmp = sum(z_traj,2);
                %z_traj = diag(param.map.C(idx_trajy,idx_trajx));
                %cost_tmp = sum(z_traj,1);
                
            case 3 % do interpolation
                Xq = traj_extended(1,:);
                Yq = traj_extended(2,:);
                Zq = interp2(map.X,map.Y,map.C,Xq,Yq);
                cost_tmp = sum(Zq,2);
        end
end
cost_traj = cost_tmp;
end
