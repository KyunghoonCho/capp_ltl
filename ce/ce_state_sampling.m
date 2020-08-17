function  z = ce_state_sampling(s_mu,s_sigma,dim,m,boundary)
% Sample (new) primitives which are parameters of trajectory.

% Input
%   s_mu:    mean of guassian var
%   s_sigma: variance of guassian var
%   dim: dimension of primitive
%   m: number of primitive
%   boundary: boundary of the workspace
%       boundary(1,1) = xmin,   boundary(1,2) = xmax.
%       boundary(2,1) = ymin,   boundary(2,2) = ymax.

num_total = dim * m;
while(1)
    z = zeros(1,2*m);
    for i = 1:m
        ind_1x2 = (i-1)*dim + (1:dim);                      % ind = [2*i-1, 2*i]; [1, 2] [3, 4] ...
        s_mu_tmp = s_mu(ind_1x2,1);
        s_sigma_tmp = s_sigma(ind_1x2,ind_1x2);
        
        z(1,ind_1x2) = mvnrnd(s_mu_tmp, s_sigma_tmp, 1);        
        % z(1,ind_1x2) = ce_mvnrnd_simple(s_mu_tmp', s_sigma_tmp, 1);        
    end
    
    % check boundary
    flag = 0;
    if ~isempty(find(z(1:2:num_total) < boundary(1,1),1))   % if x is less than min_x.
        flag = 1;
    end
    if ~isempty(find(z(2:2:num_total) < boundary(2,1),1))   % if y is less than min_y.
        flag = 1;
    end
    if ~isempty(find(z(1:2:num_total) > boundary(1,2),1))   % if x is greater than max_x.
        flag = 1;
    end
    if ~isempty(find(z(2:2:num_total) > boundary(2,2),1))   % if y is greater than max_y.
        flag = 1;
    end
    if(flag == 0)
        break;
    end
end

end
