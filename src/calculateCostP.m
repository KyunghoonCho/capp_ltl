% Compute cost of single point.
function cost_out = calculateCostP(x,map,cost_type)

switch cost_type
    case 'distance'
        cost_out = 0;
        
    case 'costmap'
        cost_out = interp2(map.X,map.Y,map.C,x(1),x(2));
end

end
