% Extend tree (or graph structure).
%   Input
%       p: current point
%       tp: target point
%       R : dynamic info
%
%   Output
%       pseg: path segment
%       u   : control segment
function [pseg,u] = extend_rewire_mex(cp,tp,R_dynamic,R_dim_x,R_dim_u,R_urange,R_dt,R_rho)

switch R_dynamic
    case 1 % 'pointmass'
        diff = tp(1:2,1) - cp(1:2,1);
        if(norm(diff,2)<1e-5)
            pseg = [];
            u = [];
            return;
        end
        if(isequal(R_urange(:,1),-1*R_urange(:,2)) && R_dim_x == R_dim_u)
            tmax = max(abs(diff./R_urange(:,2)));
            tN  = ceil(tmax/R_dt);
            t_u = [R_dt*ones(1,tN-1) ...
                (tmax - (tN-1)*R_dt)];
            t_array = [R_dt*(0:1:(tN-1)) tmax];
            
            u_tmp   = diff/tmax;
            u       = [repmat(u_tmp,1,tN); t_u];
            pseg    = repmat(cp,1,tN+1) + repmat(t_array,R_dim_u,1).*[zeros(R_dim_u,1) u(1:R_dim_u,:)];
        else
            pseg    = [cp tp];
            u       = tp - cp;
        end
        
    case {2,3,4} % {'unicycle','unicycle_part','unicycle_full'}
        [path_qi,path_rho,path_param,path_type,isFail] = dubins_init_mex(cp',tp',R_rho);
                
        if(isFail == 1)
            pseg = [];
            u = [];
            return;
        else
            inputseg = dubins_control_mex(path_qi,path_rho,path_param,path_type);
            [~,idx_zero] = find(inputseg(3,:)==0);
            if(~isempty(idx_zero))
                pseg = [];
                u = [];
                return;
            end
            
            u = zeros(R_dim_u+1,2000);
            u_len = 0;
            for nidx_seg = 1:1:size(inputseg,2)
                tN = ceil(inputseg(3,nidx_seg)/R_dt);
                t_u = [R_dt*ones(1,tN-1) ...
                    (inputseg(3,nidx_seg) - (tN-1)*R_dt)];
                
                idx_range = (u_len+1):1:(u_len+tN);
                u(:,idx_range) = [inputseg(1,nidx_seg)*ones(1,tN); ...
                    inputseg(2,nidx_seg)*ones(1,tN); ...
                    t_u];
                u_len = u_len + tN;
            end
            u = u(:,1:u_len);
            pseg = control_by_input_mex(cp,u,R_dim_x,R_dynamic);
        end
    otherwise
        pseg = [];
        u = [];
        
end % switch

end
