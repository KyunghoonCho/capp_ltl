% Rewire two points.
function [pseg,u] = extendRewire(cp,tp,R)
% p     : current point
% tp    : target point
% R     : dynamic info
%
% pseg  : path segment
% u     : control segment

switch R.dynamic
    case 'pointmass'
        diff = tp(1:2,1) - cp(1:2,1);
        if(norm(diff,2)<1e-5)
            pseg = [];
            u = [];
            return;
        end
        if(isequal(R.u_range(:,1),-1*R.u_range(:,2)) && R.dim_x == R.dim_u)
           tmax = max(abs(diff./R.u_range(:,2)));
            
            tN = ceil(tmax/R.dt);
            t_u = [R.dt*ones(1,tN-1) ...
                        (tmax - (tN-1)*R.dt)];
            t_array = [R.dt*(0:1:(tN-1)) tmax];
            
            
            u_tmp = diff/tmax;
            u = [repmat(u_tmp,1,tN); t_u];
            pseg    = repmat(cp,1,tN+1) + repmat(t_array,R.dim_u,1).*[zeros(R.dim_u,1) u(1:R.dim_u,:)];
        else
            % skip
        end
        
    case {'unicycle','unicycle_part','unicycle_full'}
        [path_tmp,isFail] = dubinsInit(cp',tp',R.rho);
        
        if(isFail == 1)
            pseg = [];
            u = [];
            return;
        else
            inputseg = dubinsControl(path_tmp);
            [~,idx_zero] = find(inputseg(3,:)==0);
            if(~isempty(idx_zero))
                pseg = [];
                u = [];
                return;
            end
            
            u = zeros(R.dim_u+1,2000);
            u_len = 0;
            for nidx_seg = 1:1:size(inputseg,2)
                tN = ceil(inputseg(3,nidx_seg)/R.dt);
                t_u = [R.dt*ones(1,tN-1) ...
                    (inputseg(3,nidx_seg) - (tN-1)*R.dt)];
                
                idx_range = (u_len+1):1:(u_len+tN);
                u(:,idx_range) = [inputseg(1,nidx_seg)*ones(1,tN); ...
                    inputseg(2,nidx_seg)*ones(1,tN); ...
                    t_u];
                u_len = u_len + tN;
            end
            u = u(:,1:u_len);
            pseg = controlByInput(cp, u, R);
        end
        
%         figure(4); clf; hold on;
%         plot(pseg(1,:),pseg(2,:),'c-');
%         plot(cp(1),cp(2),'rs');
%         plot(tp(1),tp(2),'bs');
        
end % switch

end
