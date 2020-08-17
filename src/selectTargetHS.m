% Select target highlevel state.
function idx_sel = selectTargetHS(DP,idx_HP,prob_ri)
% area_HP = DP.area_D(1,idx_HP);
% Prob = area_HP/sum(area_HP,2);
    
if (rand <= prob_ri) && (idx_HP(end) >= DP.index_RI)
    idx_sel = idx_HP(end);
else
    prob = ones(1,size(idx_HP,2))/size(idx_HP,2);
    idx_sel = idx_HP(find(rand<cumsum(prob),1,'first'));
end

end
