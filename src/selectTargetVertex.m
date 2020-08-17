% Select target vertex from selected region.
function v = selectTargetVertex(D,idx_HS,R)
p_cell = D{idx_HS}.p;

ws_range = zeros(R.dim_ws,2);
for nidx_ws = 1:1:R.dim_ws
    ws_range(nidx_ws,:) = [min(p_cell(:,nidx_ws)) max(p_cell(:,nidx_ws))];
end

cnt_while = 0;
while(1)
    cnt_while = cnt_while + 1;
    
    v_ws    = ws_range(:,1) + (ws_range(:,2) - ws_range(:,1)).*rand(R.dim_ws,1);
    v_p = v_ws;
    
    switch R.dim_ws
        case 2
            p_polygon   = [p_cell; p_cell(1,:)]';
            is_valid    = inpolygon(v_p(1),v_p(2),p_polygon(1,:),p_polygon(2,:));
        case 3
            is_valid    = inhull(v_p',p_cell); 
    end
    
    if(is_valid)
        break;
    end
    
    if(cnt_while>50)
        v_p = mean(p_cell,1);
        break;
    end
end

v = v_p;

end
