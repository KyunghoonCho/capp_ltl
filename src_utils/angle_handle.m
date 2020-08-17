% Handle angel (radian).
function qnew = angle_handle(q)
qnew = q;
idx_found_upper = q>pi;
idx_found_lower = q<-pi;
qnew(idx_found_upper)= q(idx_found_upper) - 2*pi;
qnew(idx_found_lower)= q(idx_found_lower) + 2*pi;
end