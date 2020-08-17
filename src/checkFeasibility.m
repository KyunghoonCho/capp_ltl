% Check whether pseg is feasiblity or not.
function is_unsafe = checkFeasibility(pseg,interval,R,map,use_mex)

is_unsafe = 0;

% if pseg is empty...
if(isempty(pseg))
    is_unsafe = 1;
    return;
end

% boundary check
for nidx_x = 1:1:R.dim_x
    [~,idx_nvalid_x] = find(pseg(nidx_x,:)>R.x_range(nidx_x,2) | pseg(nidx_x,:)<R.x_range(nidx_x,1));
    if(~isempty(idx_nvalid_x))
        is_unsafe = 1;
        return;
    end
end

% check pseg is not in unfeasible regions (obstacles)
if use_mex
    pseg_dense = make_denseTrajectory_mex(pseg(1:2,:),interval,R.dim_ws);
else
    pseg_dense = make_denseTrajectory(pseg(1:2,:),interval,R.dim_ws);
end
    
switch R.dim_ws
    case 2
        c_unfeas = interp2(map.X,map.Y,map.C_unfeas,pseg_dense(1,:),pseg_dense(2,:));
        [~,idx_obs] = find(c_unfeas>=0.02);
        if(~isempty(idx_obs))
            is_unsafe = 1;
        end
    case 3
        % skip
end


end
