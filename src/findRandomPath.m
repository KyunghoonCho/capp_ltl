% Find random path.
%   Input:
%       T : transitoin matrix (N by N matrix)
%       source : start index (1<=source<=N)\
%   output:
%       disc : whether a vertex is discovered or not
%       prev : index of the previous vertex
function [disc, prev] = findRandomPath(T,source)
N = size(T,1);          % number of nodes in graph
disc = zeros(1,N);
prev = zeros(1,N);

S = source;             % stack of the current vertex
Prev = 0;               % stack of the current vertex
while(~isempty(S))
    v = S(1,end);
    S(end) = [];
    pv = Prev(1,end);
    Prev(end) = [];
    % if v is not labeled as discovered
    if(disc(1,v)==0)
       disc(1,v) = 1;
       prev(1,v) = pv;
       [~,idx_v_neighbor] = find(T(v,:)>0);
       S = [S idx_v_neighbor]; %#ok<AGROW>
       Prev = [Prev v*ones(1,size(idx_v_neighbor,2))]; %#ok<AGROW>
    end
end

end
