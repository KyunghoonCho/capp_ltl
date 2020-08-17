% Check control.
function is_unsafe = ce_check_control_mex(useg,R_dim_u,R_urange)
is_unsafe = 0;
if(isempty(useg))
    return;
end
for nidx_u = 1:1:R_dim_u
    [~,idx_nvalid] = find(useg(nidx_u,:)>R_urange(nidx_u,2) | useg(nidx_u,:)<R_urange(nidx_u,1));
    if(~isempty(idx_nvalid))
        is_unsafe = 1;
        return;
    end
end

end

