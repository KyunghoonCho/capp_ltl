% Create unfeasibility (obstalce) map.
% (1 for unfeasible (obstacle) region and 0 for otherwise)
function C_unfeas = createObsMap(map,P)

C_unfeas = zeros(size(map.X));
for nidx_row = 1:1:size(C_unfeas,1)
    for nidx_col = 1:1:size(C_unfeas,2)
        x_p = map.X(nidx_row,nidx_col);
        y_p = map.Y(nidx_row,nidx_col);
        for nidx_p = 1:1:size(P,2)
            label_p = P{1,nidx_p}.label;
            if(strcmp(label_p,'o'))
                p_polygon = [P{1,nidx_p}.points; P{1,nidx_p}.points(1,:)]';
                if(inpolygon(x_p, y_p, p_polygon(1,:), p_polygon(2,:)))
                    C_unfeas(nidx_row,nidx_col)=1;
                end
            end
        end
    end
end

end
