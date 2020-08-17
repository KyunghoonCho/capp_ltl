% Decompose trajectory into segments.
function [idx_p_segment,idx_c_segment] = decompose_trajectory(path_gen,idx_ws,segment_distance)

% find distance between two vertices
dtmp_array = zeros(1,size(path_gen,2)-1);
for nidx_p = 2:1:size(path_gen,2)
    p1 = path_gen(idx_ws,nidx_p);
    p2 = path_gen(idx_ws,nidx_p-1);
    dtmp_array(1,nidx_p-1) = norm(p1-p2);
end
dist_total  = sum(dtmp_array,2);
N_seg_pre   = ceil(dist_total/segment_distance);

% find end-index of each segment
idx_seg_array = zeros(1,N_seg_pre);
cnt_seg     = 0;
dist_sum    = 0;
for nidx_p = 2:1:size(path_gen,2)
    dist_sum = dist_sum + dtmp_array(1,nidx_p-1);
    if(dist_sum>=(cnt_seg+1)*segment_distance)
        if(cnt_seg > 1)
            if(idx_seg_array(1,cnt_seg) == nidx_p)
                continue;
            end
        end
        cnt_seg = cnt_seg + 1;
        idx_seg_array(1,cnt_seg) = nidx_p;
        if(cnt_seg == N_seg_pre-1)
            break;
        end
    end
end
if(cnt_seg>=1)
    if(idx_seg_array(:,cnt_seg) < size(path_gen,2))
        cnt_seg = cnt_seg + 1;
        idx_seg_array(:,cnt_seg) = size(path_gen,2);
    end
else
    cnt_seg = cnt_seg + 1;
    idx_seg_array(:,cnt_seg) = size(path_gen,2);
end
idx_seg_array = idx_seg_array(:,1:cnt_seg);

% update idx_p_segment & idx_c_segment
N_segment       = size(idx_seg_array,2);
idx_p_segment   = zeros(N_segment,2);
idx_c_segment    = zeros(N_segment,2);
for nidx_seg = 1:1:N_segment
    if(nidx_seg == 1)
        idx_st = 1;
        idx_ctr_st = 1;
    else
        idx_st = idx_seg_array(1,nidx_seg-1);
        idx_ctr_st = idx_seg_array(1,nidx_seg-1);
    end
    idx_p_segment(nidx_seg,:) = [idx_st idx_seg_array(1,nidx_seg)];
    idx_c_segment(nidx_seg,:) = [idx_ctr_st (idx_seg_array(1,nidx_seg)-1)];
end

end
