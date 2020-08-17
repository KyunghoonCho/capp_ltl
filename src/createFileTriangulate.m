% Create triangulate file.
function createFileTriangulate(P,fName)
N = size(P,2);
% write file
fid = fopen(fName,'w');
if(fid~=-1)
    fprintf(fid,'%d\n\n',N);
end
sizeP = size(P,2);
for np = 1:1:sizeP
    if(fid~=-1)
        fprintf(fid,'%d\n',P{np}.numpoints);
        size_p = size(P{np}.points,1);
        for npp = 1:1:size_p
            pointArray = P{np}.points;
            fprintf(fid,'%.1f %.1f\n',pointArray(npp,1),pointArray(npp,2));
        end
        if(np<sizeP)
            fprintf(fid,'\n');
        end
    end
end
if(fid~=-1)
    fclose(fid);
end
end
