% Check whether pseg is feasiblity or not.
function is_unsafe = ce_check_feasibility_mex(pseg,interval,R_idx_ws,R_dim_ws,R_dim_x,R_xrange, ...
    map_X,map_Y,map_C_unfeas)

is_unsafe   = 0;


% if pseg is empty...
if(isempty(pseg))
    is_unsafe = 1;
    return;
end

% boundary check
for nidx_x = 1:1:R_dim_x
    [~,idx_nvalid_x] = find(pseg(nidx_x,:)>R_xrange(nidx_x,2) | pseg(nidx_x,:)<R_xrange(nidx_x,1));
    if(~isempty(idx_nvalid_x))
        is_unsafe = 1;
        return;
    end
end

% check pseg is not in unfeasible regions (obstacles)
pseg_dense = ce_make_denseTrajectory(pseg(R_idx_ws,:),interval,R_dim_ws);
switch R_dim_ws
    case 2
        c_unfeas    = interp2(map_X,map_Y,map_C_unfeas,pseg_dense(1,:),pseg_dense(2,:));
        [~,idx_obs] = find(c_unfeas>=0.1);
        if(~isempty(idx_obs))
            is_unsafe = 1;
        end
    case 3
        % skip
    otherwise
        % skip
end


end
