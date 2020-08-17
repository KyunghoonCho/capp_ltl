% Sample (new) primitives which are parameters of trajectory.
% dynamic: unicycle model
%
% Input
%   s_mu:    mean of guassian var
%   s_sigma: variance of guassian var
%   dim: dimension of primitive
%       2 : unicycle dynamic (only time and w are sampled and v is fixed)
%       3 : unicycle dynamic (ver2, time, v, and w are sampled)
%   m: number of primitive
%   constraint:
%       unicycle (dim 2) : [wmin wmax]
%       unicycle (dim 3) : [vmin vmax; wmin wmax]

function  z = ce_input_sampling(s_mu,s_sigma,dim,m,costraint)

num_total = dim * m;

while(1)
    z = zeros(1,2*m);
    for i = 1:m
        ind_1xdim = (i-1)*dim + (1:dim);  % ind = [2*i-1, 2*i]; [1, 2] [3, 4] ...
        s_mu_tmp = s_mu(ind_1xdim);
        s_sigma_tmp = s_sigma(ind_1xdim,ind_1xdim);
        
        %z(ind_1xdim) = mvnrnd(s_mu_tmp, s_sigma_tmp, 1);
        z(ind_1xdim) = ce_mvnrnd_simple(s_mu_tmp, s_sigma_tmp, 1);
    end
    
    flag = 0;
    if ~isempty(find(z(1:dim:num_total) < 0 ,1))                    % if time is less than zero.
        flag = 1;
    end
    
    % check constraint
    switch dim
        case 2
            if ~isempty(find(z(2:dim:num_total) > costraint(2,2),1))
                flag = 1;
            end
            if ~isempty(find(z(2:dim:num_total) < costraint(2,1),1))
                flag = 1;
            end
        case 3
            for nidx_tmp = (dim-1):1:dim
                if ~isempty(find(z(nidx_tmp:dim:num_total) > costraint(nidx_tmp-1,2),1))
                    flag = 1;
                end
                if ~isempty(find(z(nidx_tmp:dim:num_total) < costraint(nidx_tmp-1,1),1))
                    flag = 1;
                end
            end
        otherwise
            % skip
    end
    
    if flag == 0
        break;
    end
end

end
