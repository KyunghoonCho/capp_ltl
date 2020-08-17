% Update 'path' and 'cost' of child nodes of idx_root.
function T = updateChild(T,idx_root)

Q = T.e.child{1,idx_root};

while(~isempty(Q))
    idx_q = Q(1);
    Q(1) = [];
    
    idx_q_parent = T.e.parent(1,idx_q);
    path_parent = T.v.path{1,idx_q_parent};
    
    path_updated = [path_parent T.v.x(:,idx_q)];
    cost_updated = T.v.cost(1,idx_q_parent) + T.v.pseg_cost(1,idx_q);
        
    T.v.path{1,idx_q} = path_updated;
    T.v.cost(1,idx_q) = cost_updated;
    
    idx_child_array = T.e.child{1,idx_q};
    if(~isempty(idx_child_array))
        Q = [Q idx_child_array]; %#ok<AGROW>
    end
end

end
