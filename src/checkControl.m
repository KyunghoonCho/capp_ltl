% Check control is valid.
function is_unsafe = checkControl(useg,R)
is_unsafe = 0;
if(isempty(useg))
    return;
end
for nidx_u = 1:1:R.dim_u
    [~,idx_nvalid] = find(useg(nidx_u,:)>R.u_range(nidx_u,2) | useg(nidx_u,:)<R.u_range(nidx_u,1));
    if(~isempty(idx_nvalid))
        is_unsafe = 1;
        return;
    end
end

end

