% Do dubins control.
function input_c = dubins_control_mex(path_qi,path_rho,path_param,path_type)
% The segment types for each of the Path types
%
%   'L_SEG' -> 1;   'S_SEG' -> 2;   'R_SEG' -> 3;
%
%   'L_SEG'    'S_SEG'    'L_SEG'           1   2   1
%   'L_SEG'    'S_SEG'    'R_SEG'           1   2   3
%   'R_SEG'    'S_SEG'    'L_SEG'     =>    3   2   1
%   'R_SEG'    'S_SEG'    'R_SEG'           3   2   3
%   'R_SEG'    'L_SEG'    'R_SEG'           3   1   3
%   'L_SEG'    'R_SEG'    'L_SEG'           1   3   1

DIRDATA = [1 2 1; 1 2 3; 3 2 1; 3 2 3; 3 1 3; 1 3 1];

% Generate the target configuration
type = DIRDATA(path_type,:);
% p1  = path_param(1,1);
% p2  = path_param(1,2);
% p3  = path_param(1,3);

% The translated initial configuration
% qi = [ 0 0 path_qi(3)];

% end of segment 1
% q1 = dubins_segment(p1,qi,type(1));
% end of segment 2
% q2 = dubins_segment(p2,q1,type(2));
% end of segment 3
% q3 = dubins_segment(p3,q2,type(3));

% restore control
u_linear = 1;

control_v = zeros(1,3);
control_w = zeros(1,3);
control_t = zeros(1,3);
for nidx_t = 1:1:3
    switch type(1,nidx_t)
        case 1  % 'L_SEG'
            wdir = 1;
        case 3  % 'R_SEG'
            wdir = -1;
        case 2  % 'S_SEG'
            wdir = 0;
        otherwise % 'L_SEG'
            wdir = 1;
    end
    
    if(wdir == 0)
        control_v(1,nidx_t) = path_rho;
        control_w(1,nidx_t) = 0;
        control_t(1,nidx_t) = path_param(1,nidx_t);
    else
        control_v(1,nidx_t) = u_linear;
        control_w(1,nidx_t) = wdir*u_linear/path_rho;
        control_t(1,nidx_t) = path_param(1,nidx_t)/abs(control_w(1,nidx_t));
    end
        
end
input_c = [control_v; control_w; control_t];
% input_c = make_denseControl(input_c,0.1);
end

% function qt = dubins_segment(t,qi,type)
% 
% switch type
%     case 1 % 'L_SEG'
%         qt = qi + [(sin(qi(3)+t)-sin(qi(3))) (cos(qi(3))-cos(qi(3)+t)) t];
%     case 3 % 'R_SEG'
%         qt = qi + [(-sin(qi(3)-t) + sin(qi(3))) (cos(qi(3)-t) - cos(qi(3))) -t];
%     case 2 % 'S_SEG'
%         qt = qi + [(cos(qi(3))*t) (sin(qi(3))*t) 0];
%         
%     otherwise % 'L_SEG'
%         qt = qi + [(sin(qi(3)+t)-sin(qi(3))) (cos(qi(3))-cos(qi(3)+t)) t];
% end
% end

