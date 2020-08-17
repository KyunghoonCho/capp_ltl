% Find path from control-input.
function [ path_out ] = controlByInput(sp, input_c, param)
path_out = zeros(param.dim_x,1+size(input_c,2));
p_index = 1;
path_out(:,p_index) = sp;
x_current = sp;

for k = 1:size(input_c,2)
    switch param.dynamic
        case 'pointmass'
            vx = input_c(1,k);
            vy = input_c(2,k);
            dt = input_c(3,k);
            
            x_updated = x_current + [vx; vy]*dt;
            
        case {'unicycle_part','unicycle_full'}
            v = input_c(1,k);
            w = input_c(2,k);
            dt = input_c(3,k);
            theta = x_current(3);
            if(w==0)
                x_updated = x_current+[v*cos(theta)*dt; v*sin(theta)*dt; 0];
            else
                x_updated = x_current+[-v/w*sin(theta)+v/w*sin(theta+w*dt);...
                    v/w*cos(theta)-v/w*cos(theta+w*dt);...
                    w*dt];
            end
            
            if(x_updated(3)>pi)
                x_updated(3) = x_updated(3) - 2*pi;
            end
            
            if(x_updated(3)<-pi)
                x_updated(3) = x_updated(3) + 2*pi;
            end
            
    end
    p_index = p_index + 1;
    path_out(:,p_index) = x_updated;
    x_current = x_updated;
end

end

