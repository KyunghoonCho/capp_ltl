%% DIJKSTRA
%   Input:
%       T : transitoin matrix (N by N matrix)
%       L : length matrix (N by N matrix)
%       source : start index (1<=source<=N)
%   Output:
%       dist : distance form source to node
%       prev : previous node in optimal path

function [dist, prev] = dijkstra(T,L,source) %# codegen
    N = size(T,1);      % number of nodes in graph
    dist = zeros(1,N);  % distance from source to node
    prev = zeros(1,N);  % previous node in optimal path initialization
    
    if(~isequal(size(T),size(L))) % when size of two matrix have a different size.
        return;
    end
    
    % Initialization
    for nv = 1:1:N      % for each vertex v in Graph
        if(nv~=source)
            dist(1,nv) = inf;
        end
    end
    Q = 1:N;            % all nodes initially in Q (unvisited nodes)
    
    while(~isempty(Q))
        [~,u_tmp] = min(dist(1,Q));  % vertex in Q with min dist[u] (source node in first case)
        u = Q(1,u_tmp);
        Q(u_tmp) = [];        % remove u from Q
        
        [~,index_u_neighbor] = find(T(u,:)>0);
        
        for nu = 1:1:size(index_u_neighbor,2)   % where v is still in Q
            v = index_u_neighbor(1,nu);
            if(~ismember(v,Q))
                continue;
            end
            
            alt = dist(1,u) + L(u,v);
            if(alt<dist(1,v))
                dist(1,v) = alt;
                prev(1,v) = u;
            end
        end
    end
end
