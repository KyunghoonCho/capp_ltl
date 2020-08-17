% Select initial highlevel state.
function [z_from,d_from,g_from] = selectInitialHS(T,w,num_Z,accepting_state_ids)

num_node = T.num_node;

z_avail = T.v.alpha(1,1:num_node);
d_avail = T.v.d(1,1:num_node);

% remove accepting states
idx_z_accept = zeros(1,num_node);
cnt_z_accept = 0;
for nidx_za = 1:1:size(accepting_state_ids,2)
    accepting_state_ids_sel = accepting_state_ids(1,nidx_za);
    [~,idx_tmp] = find(z_avail==accepting_state_ids_sel);
    idx_z_accept(1,(cnt_z_accept+1):1:(cnt_z_accept+size(idx_tmp,2))) = idx_tmp;
    cnt_z_accept = cnt_z_accept + size(idx_tmp,2);
end
idx_z_accept = idx_z_accept(1,1:cnt_z_accept);
if(~isempty(idx_z_accept))
    z_avail(idx_z_accept) = [];
    d_avail(idx_z_accept) = [];
    num_node = num_node - size(idx_z_accept,2);
end

g_avail = num_Z*(d_avail-ones(1,num_node)) + z_avail;
g_avail = unique(g_avail);

w_avail = w(1,g_avail);
sum_w_avail = sum(w_avail,2);
if(sum_w_avail==0)
    Prob = ones(1,size(g_avail,2));
else
    Prob = w_avail/sum_w_avail;
end

g_from = g_avail(find(rand<cumsum(Prob),1,'first'));
if(g_from<=0)
    fprintf('error!\n');
end

z_from = mod(g_from,num_Z);
if(z_from==0)
    z_from = num_Z;
end
d_from = (g_from-z_from)/num_Z+1;

end
