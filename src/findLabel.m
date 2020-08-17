% Find the label of the point.
%   Input
%       DP: discrete (high-level) planner
%       x_state: a physical state
function label = findLabel(DP,x_state)

label   = 'w';
dim_ws  = size(DP.D{2}.p,2);

mode = 2;
switch mode
    case 1
        for nidx_i = DP.index_RI:1:size(DP.D,2)
            switch size(x_state,1)
                case 2
                    p_polygon = [DP.D{nidx_i}.p; DP.D{nidx_i}.p(1,:)]';
                    in  = inpolygon(x_state(1),x_state(2),p_polygon(1,:),p_polygon(2,:));
                    
                case 3
                    in  = inhull(x_state(1:dim_ws,1)',DP.D{nidx_i}.p);
            end
            
            if(in)
                label = DP.D{nidx_i}.label;
                return;
            end
        end
        
    case 2
        for nidx_i = DP.index_RI:1:size(DP.D,2)
            x_range = DP.D{nidx_i}.p_range;
            
            in = (x_state(1:dim_ws,1) <= x_range(:,2)) & (x_state(1:dim_ws,1) >= x_range(:,1));
            
            if(sum(in,1) == dim_ws)
                label = DP.D{nidx_i}.label;
                return;
            end
        end     
end

end
