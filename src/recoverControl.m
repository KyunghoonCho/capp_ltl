% Recover control.
function [input_out] = recoverControl(edge,input,leaf_index)
input_out = [];
index = leaf_index;
while(index>0)
    input_sel = input{1,index};
    if(~isempty(input_sel))
        input_out = [input_sel input_out]; %#ok<AGROW>
    end
    index = edge(1,index);
end
