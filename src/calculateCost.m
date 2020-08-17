% Compute cost of trajectory.
function cost_out = calculateCost(traj,map,interval,cost_type,R,mode,use_mex)

switch cost_type
    case 'distance'
        if(size(traj,1)<R.dim_x)
            traj_tmp = [traj; zeros(R.dim_x-size(traj,1),size(traj,2))];
        else
            traj_tmp = traj;
        end
        if(size(traj,2)>=2)
            cost_out = R.dist_fun(traj_tmp(:,2:end),traj_tmp(:,1:end-1));
        else
            cost_out = 0;
        end
        
    case 'costmap'
        choose_method = 3;
        cost_out = 0;
        
        if(size(traj,2)<2)
            return;
        end
        
        % make_dense_trajectory
        if use_mex
            traj_extended_tmp = make_denseTrajectory_mex(traj(1:R.dim_ws,:),interval,R.dim_ws);
        else
            traj_extended_tmp = make_denseTrajectory(traj(1:R.dim_ws,:),interval,R.dim_ws);
        end
        if(size(traj_extended_tmp,2)>=2 && mode==1)
            % remove initial state. (due to duplication issue...)
            traj_extended = traj_extended_tmp(:,2:end);
        else
            traj_extended = traj_extended_tmp;
        end
        
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
                cost_out = sum(z_traj,2);
                
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
                
                cost_out = sum(z_traj,2);
                %z_traj = diag(param.map.C(idx_trajy,idx_trajx));
                %cost_tmp = sum(z_traj,1);
                
            case 3 % do interpolation
                Xq = traj_extended(1,:);
                Yq = traj_extended(2,:);
                Zq = interp2(map.X,map.Y,map.C,Xq,Yq);
                cost_out = sum(Zq,2);
        end
end

end
