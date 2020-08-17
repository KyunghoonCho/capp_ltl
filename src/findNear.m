% Find near vertices.
function ind_near = findNear(T,p,q,idx_ws,dist_near)
ind_near = [];
x_avail = T.v.x(idx_ws,1:T.num_node);
q_avail = T.v.alpha(1,1:T.num_node);
diffvec = repmat(p(idx_ws),1,size(x_avail,2))-x_avail;
diffvec_scalar = sqrt(sum(diffvec.^2,1));
[~,ind_near1] = find(diffvec_scalar<=dist_near);
if(isempty(ind_near1))
    return;
end

if(isempty(q))
    ind_near = ind_near1;
else
    [~,ind_near2] = find(q_avail(1,ind_near1)==q);
    if(isempty(ind_near2))
        return;
    end
    ind_near = ind_near1(1,ind_near2);
end

end
