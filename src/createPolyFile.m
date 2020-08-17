% Create poly-file for mesh generation.
function createPolyFile(P,fName)
total_num_points = 0;
total_num_seg = 0;
for nidx_P = 1:1:size(P,2)
    total_num_points = total_num_points + size(P{1,nidx_P}.points,1);
    if(nidx_P>1)
        total_num_seg = total_num_seg + size(P{1,nidx_P}.points,1);
    end
end

% write file
fid = fopen(fName,'w');
if(fid~=-1)
    fprintf(fid,'%s\n','# A workspace in 2D');
    fprintf(fid,'%d 2 0 0\n',total_num_points);
    
    % write vertices
    idx_point = 0;
    idx_points_array = cell(1,size(P,2));
    for nidx_P = 1:1:size(P,2)
        if(nidx_P==1)
            fprintf(fid,'%s\n','# Outer box:');
        elseif(nidx_P==2)
            fprintf(fid,'%s\n','# Inner box:');
        else
            % skip
        end
        
        for ndix_point = 1:1:size(P{1,nidx_P}.points,1)
            idx_point = idx_point+1;
            point_x = P{1,nidx_P}.points(ndix_point,1);
            point_y = P{1,nidx_P}.points(ndix_point,2);
            fprintf(fid,' %d %f %f %d\n',idx_point,point_x,point_y,0);
            
            idx_points_array{1,nidx_P} = [idx_points_array{1,nidx_P} idx_point];
        end
    end
    
    % write edges (segment)
    idx_seg = 0;
    fprintf(fid,'%s\n','# segments');
    fprintf(fid,'%d %d\n',total_num_seg,0);
    for nidx_P = 2:1:size(P,2)
        idx_points_array_sel = idx_points_array{1,nidx_P};
        
        for nidx_seg = 1:1:size(idx_points_array_sel,2)
            idx_seg = idx_seg + 1;
            if(nidx_seg == size(idx_points_array_sel,2))
                fprintf(fid,' %d %d %d %d\n',idx_seg,idx_points_array_sel(1,nidx_seg),idx_points_array_sel(1,1),0);
            else
                fprintf(fid,' %d %d %d %d\n',idx_seg,idx_points_array_sel(1,nidx_seg),idx_points_array_sel(1,nidx_seg+1),0);
            end
        end
    end
    
    % write hole
    idx_hole = 0;
    fprintf(fid,'%s\n','# holes');
    fprintf(fid,'%d\n',size(P,2)-1);
    for nidx_P = 2:1:size(P,2)
        idx_hole = idx_hole+1;
        x_hole = mean(P{1,nidx_P}.points(:,1),1);
        y_hole = mean(P{1,nidx_P}.points(:,2),1);
        fprintf(fid,' %d %f %f\n',idx_hole,x_hole,y_hole);
    end
    fclose(fid);
end
end