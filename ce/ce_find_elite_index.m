function sorted_idx = ce_find_elite_index(cost,N_el1,N_el2)
% find elite set
[~, sorted_T_idx] = sort(cost.terminal,'ascend');  % terminal cost filtering
filtered_env = cost.traj(sorted_T_idx(1:N_el1));   % energy cost filtering
[~, sorted_E_idx] = sort(filtered_env,'ascend');
sorted_idx = sorted_T_idx(sorted_E_idx(1:N_el2));
end