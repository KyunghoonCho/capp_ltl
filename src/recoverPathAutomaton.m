% Recover Automaton path.
function [path_out] = recoverPathAutomaton(alpha,edge,edgepath,leaf_index) %#codegen
path_out    = alpha(1,leaf_index);
index       = leaf_index;
while(1)
    index           = edge(1,index); 
    if(index <= 0)
        break;
    end
    edgepath_sel    = edgepath{1,index}(:,1:end-1);
    alpha_new       = alpha(1,index);
    path_out        = [alpha_new*ones(1,size(edgepath_sel,2)) path_out]; %#ok<AGROW>   
end
end
