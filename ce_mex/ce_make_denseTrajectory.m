% Make trajectory dense with respect to 'min_dist_interval'.
function traj_dense = ce_make_denseTrajectory(traj_in,min_dist_interval,dim_ws)

eta = 0.001;

len_traj = size(traj_in,2);
if(len_traj==1)
    traj_dense = traj_in(:,1);
else
    traj_dense = zeros(dim_ws, 100000);
    cnt_td = 0;
    
    for nidx_traj = 1:1:len_traj-1
        ps = traj_in(1:dim_ws,nidx_traj);
        pg = traj_in(1:dim_ws,nidx_traj+1);
        diff = pg-ps;
        dist = norm(diff,2);
        
        if(dist<eta)
            traj_seg = ps;
        elseif(dist>min_dist_interval*1.5)
            num_points = floor(dist/min_dist_interval);
            traj_seg = repmat(ps,1,num_points+1) + ...
                repmat(0:num_points,dim_ws,1).*repmat((min_dist_interval/dist)*diff,1,num_points+1);
            if(dist-num_points*min_dist_interval<eta)
                traj_seg = traj_seg(:,1:end-1);
            end
        else
            traj_seg = ps;
        end
                
        if(nidx_traj==(len_traj-1))
            traj_seg = cat(2, traj_seg, pg);
        end
        
        % update traj_dense
        idx_update = (cnt_td + 1):(cnt_td + size(traj_seg,2));
        traj_dense(:, idx_update) = traj_seg;
        cnt_td = cnt_td + size(traj_seg,2);
    end
    traj_dense = traj_dense(:, 1:cnt_td);
end
end
