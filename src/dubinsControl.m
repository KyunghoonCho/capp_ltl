% Dubins-control.
function input_c = dubinsControl(path)
% The segment types for each of the Path types
DIRDATA = {'L_SEG' 'S_SEG' 'L_SEG'; ...
    'L_SEG', 'S_SEG' 'R_SEG'; ...
    'R_SEG', 'S_SEG', 'L_SEG'; ...
    'R_SEG', 'S_SEG', 'R_SEG'; ...
    'R_SEG', 'L_SEG', 'R_SEG'; ...
    'L_SEG', 'R_SEG', 'L_SEG'};

% Generate the target configuration
type = DIRDATA(path.type,:);
p1 = path.param(1,1);
p2 = path.param(1,2);
p3 = path.param(1,3);

% The translated initial configuration
qi = [ 0 0 path.qi(3)];

% end of segment 1
q1 = dubins_segment(p1,qi,type{1,1});
% end of segment 2
q2 = dubins_segment(p2,q1,type{1,2});
% end of segment 3
q3 = dubins_segment(p3,q2,type{1,3});


% restore control
u_linear = 1;

control_v = zeros(1,3);
control_w = zeros(1,3);
control_t = zeros(1,3);
for nidx_t = 1:1:3
    switch type{1,nidx_t}
        case 'L_SEG'
            wdir = 1;
        case 'R_SEG'
            wdir = -1;
        case 'S_SEG'
            wdir = 0;
    end
    
    if(wdir == 0)
        control_v(1,nidx_t) = path.rho;
        control_w(1,nidx_t) = 0;
        control_t(1,nidx_t) = path.param(1,nidx_t);
    else
        control_v(1,nidx_t) = u_linear;
        control_w(1,nidx_t) = wdir*u_linear/path.rho;
        control_t(1,nidx_t) = path.param(1,nidx_t)/abs(control_w(1,nidx_t));
        
    end
        
end
input_c = [control_v; control_w; control_t];
% input_c = make_denseControl(input_c,0.1);
end

function qt = dubins_segment(t,qi,type)

switch type
    case 'L_SEG'
        qt = qi + [(sin(qi(3)+t)-sin(qi(3))) (cos(qi(3))-cos(qi(3)+t)) t];
    case 'R_SEG'
        qt = qi + [(-sin(qi(3)-t) + sin(qi(3))) (cos(qi(3)-t) - cos(qi(3))) -t];
    case 'S_SEG'
        qt = qi + [(cos(qi(3))*t) (sin(qi(3))*t) 0];
end
end

function input_out = make_denseControl(input,dt)
output_size = [ceil(input(3,1)/dt) ceil(input(3,2)/dt) ceil(input(3,3)/dt)];

input_out = zeros(3,sum(output_size,2));
idx_st = 1;

for k = 1:1:3
    if(input(3,k)/dt == floor(input(3,k)/dt))
        input_k = repmat([input(1:2,k); dt],1,output_size(k));
    else
        dt_diff = input(3,k) - dt*floor(input(3,k)/dt);
        input_k = [repmat([input(1:2,k); dt],1,output_size(k)-1) [input(1:2,k); dt_diff]];
    end
    input_out(:,idx_st:idx_st+size(input_k,2)-1) = input_k;
    idx_st = idx_st+size(input_k,2);
end

end

