% Initialization for dubins-control.
function [path,errorout] = dubinsInit(q0,q1,rho)

dx = q1(1) - q0(1);
dy = q1(2) - q0(2);
D = sqrt(dx*dx + dy*dy);
d = D / rho;
if( rho<0 )
    fprintf('ERROR\n');
    errorout = 1; % the rho value is invalid
    path = struct;
else
    theta = mod(atan2(dy,dx),2*pi);
    alpha = mod((q0(3)-theta),2*pi);
    beta = mod((q1(3)-theta),2*pi);
    
    path.qi = q0;
    path.rho = rho;
    
    [path,errorout] = dubins_init_normalised(alpha, beta, d, path);
end
end

function [sa, sb, ca, cb, c_ab] = UNPACK_INPUTS(alpha,beta)
sa = sin(alpha);
sb = sin(beta);
ca = cos(alpha);
cb = cos(beta);
c_ab = cos(alpha-beta);
end

function [output] = PACK_OUTPUTS(t,p,q)
output = [t,p,q];
end

function [path,errorout] = dubins_init_normalised(alpha, beta, d, path)
best_cost = inf;
best_word = -1;

for nidx_type = 1:1:6
    switch nidx_type
        % choose type
        case 1
            [param, error] = dubins_LSL(alpha, beta, d);
        case 2
            [param, error] = dubins_LSR(alpha, beta, d);
        case 3
            [param, error] = dubins_RSL(alpha, beta, d);
        case 4
            [param, error] = dubins_RSR(alpha, beta, d);
        case 5
            [param, error] = dubins_RLR(alpha, beta, d);
        case 6
            [param, error] = dubins_LRL(alpha, beta, d);
    end
    if(error == 0)
        cost = sum(param,2);
        if(cost<best_cost)
            best_word = nidx_type;
            best_cost = cost;
            path.param = param;
            path.type = nidx_type;
        end
    end
end

if(best_word == -1)
    errorout = 1;
else
    errorout = 0;
end


end

function [output, error] = dubins_LSL(alpha, beta, d)
[sa, sb, ca, cb, c_ab] = UNPACK_INPUTS(alpha,beta);
tmp0 = d+sa-sb;
p_squared = 2 + (d*d) -(2*c_ab) + (2*d*(sa - sb));
if(p_squared<0)
    error = 4; % no connection between configurations with this word
    output = [];
else
    error = 0; % no error;
    tmp1 = atan2((cb-ca),tmp0);
    t = mod(-alpha+tmp1,2*pi);
    p = sqrt( p_squared );
    q = mod(beta - tmp1,2*pi);
    [output] = PACK_OUTPUTS(t,p,q);
end
end

function [output, error] = dubins_LSR(alpha, beta, d)
[sa, sb, ca, cb, c_ab] = UNPACK_INPUTS(alpha,beta);
p_squared = -2 + (d*d) + (2*c_ab) + (2*d*(sa+sb));
if(p_squared<0)
    error = 4; % no connection between configurations with this word
    output = [];
else
    error = 0; % no error;
    p = sqrt( p_squared );
    tmp2 = atan2( (-ca-cb), (d+sa+sb) ) - atan2(-2.0, p);
    t = mod(-alpha + tmp2,2*pi);
    q = mod(-mod(beta,2*pi) + tmp2,2*pi);
    [output] = PACK_OUTPUTS(t,p,q);
end
end

function [output, error] = dubins_RSL(alpha, beta, d)
[sa, sb, ca, cb, c_ab] = UNPACK_INPUTS(alpha,beta);
p_squared = (d*d) -2 + (2*c_ab) - (2*d*(sa+sb));
if(p_squared<0)
    error = 4; % no connection between configurations with this word
    output = [];
else
    error = 0; % no error;
    p = sqrt( p_squared );
    tmp2 = atan2( (ca+cb), (d-sa-sb) ) - atan2(2.0, p);
    t = mod(alpha - tmp2,2*pi);
    q = mod(beta - tmp2,2*pi);
    [output] = PACK_OUTPUTS(t,p,q);
end
end

function [output, error] = dubins_RSR(alpha, beta, d)
[sa, sb, ca, cb, c_ab] = UNPACK_INPUTS(alpha,beta);
tmp0 = d-sa+sb;
p_squared = 2 + (d*d) -(2*c_ab) + (2*d*(sb-sa));
if(p_squared<0)
    error = 4; % no connection between configurations with this word
    output = [];
else
    error = 0; % no error;
    tmp1 = atan2((ca-cb),tmp0 );
    t = mod(alpha - tmp1,2*pi);
    p = sqrt(p_squared);
    q = mod(-beta + tmp1,2*pi);
    [output] = PACK_OUTPUTS(t,p,q);
end
end

function [output, error] = dubins_RLR(alpha, beta, d)
[sa, sb, ca, cb, c_ab] = UNPACK_INPUTS(alpha,beta);
tmp_rlr = (6 - d*d + 2*c_ab + 2*d*(sa-sb)) / 8;
if(abs(tmp_rlr)>1)
    error = 4; % no connection between configurations with this word
    output = [];
else
    error = 0; % no error;
    p = mod(2*pi - acos( tmp_rlr ),2*pi);
    t = mod(alpha - atan2(ca-cb, d-sa+sb) + mod(p/2,2*pi),2*pi);
    q = mod(alpha - beta - t + mod(p,2*pi),2*pi);
    [output] = PACK_OUTPUTS(t,p,q);
end
end

function [output, error] = dubins_LRL(alpha, beta, d)
[sa, sb, ca, cb, c_ab] = UNPACK_INPUTS(alpha,beta);
tmp_lrl = (6 - d*d + 2*c_ab + 2*d*(- sa + sb)) / 8;
if(abs(tmp_lrl)>1)
    error = 4; % no connection between configurations with this word
    output = [];
else
    error = 0; % no error;
    p = mod(2*pi - acos( tmp_lrl ),2*pi);
    t = mod(-alpha - atan2( ca-cb, d+sa-sb ) + p/2, 2*pi);
    q = mod(mod(beta,2*pi) - alpha -t + mod(p,2*pi),2*pi);
    [output] = PACK_OUTPUTS(t,p,q);
end
end

