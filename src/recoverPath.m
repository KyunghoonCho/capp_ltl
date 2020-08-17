% Recover path.
function [path_out] = recoverPath(node,edge,edgepath,leaf_index) %#codegen
path_out    = node(:,leaf_index);
index       = leaf_index;
while(index>0)
    edgepath_sel    = edgepath{1,index}(:,1:end-1);
    path_out        = [edgepath_sel path_out]; %#ok<AGROW>
    index           = edge(1,index);
end
end
