% Select a vertex in the tree(T) to extend.
function [index_selected] = selectVertex2Extend(T,R,z_sel,d_sel,p_target,probsv_rand)

[~,idx_sel] = find(T.v.alpha(1,1:T.num_node)==z_sel ...
    & T.v.d(1,1:T.num_node)==d_sel);

if(rand<probsv_rand)
    index_selected = idx_sel(1,randi(size(idx_sel,2)));
else
    mode = 1; % 1: choose nearest node, 2: choose min cost2come
    switch mode
        case 1
            x_avail = T.v.x(:,idx_sel);
            if(size(p_target,1) == R.dim_ws)
                distvec = R.dist_fun_array_ws(x_avail(R.idx_ws,:),p_target);
            else
                distvec = R.dist_fun_array(x_avail,p_target);
            end
            
            [~,idx_min] = min(distvec);
            index_selected = idx_sel(1,idx_min);
        case 2
            cost_avail = T.v.cost(1,idx_sel);
            [~,idx_min] = min(cost_avail);
            index_selected = idx_sel(1,idx_min);
    end
end

end
