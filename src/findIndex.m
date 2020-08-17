% Find the index of discrete region.
%   Input
%       discrete_region: N regions in the form of polygons (1xN cell) .p: points of vertices
%       index_RI: start index of region of 'region of interest'
%       traj: trajectory
function ind_traj = findIndex(discrete_region,index_RI,traj)

len_traj = size(traj,2);

ind_traj = -1*ones(1,len_traj);
for nidx_t = 1:1:len_traj
    x_state = traj(:,nidx_t);
    
    % check region of interest first
    for nf = index_RI:1:size(discrete_region,2)
        p_polygon = [discrete_region{nf}.p; discrete_region{nf}.p(1,:)]';
        %in = inhull([x_state(1) x_state(2)],discrete_region{nf}.p);
        in = inpolygon(x_state(1),x_state(2),p_polygon(1,:),p_polygon(2,:));
        if(in)
            ind_traj(1,nidx_t) = nf;
            continue;
        end
    end
    
    for nf = 1:1:(index_RI-1)
        p_polygon = [discrete_region{nf}.p; discrete_region{nf}.p(1,:)]';
        %in = inhull([x_state(1) x_state(2)],discrete_region{nf}.p);
        in = inpolygon(x_state(1),x_state(2),p_polygon(1,:),p_polygon(2,:));
        if(in)
            ind_traj(1,nidx_t) = nf;
            continue;
        end
    end
end

end
