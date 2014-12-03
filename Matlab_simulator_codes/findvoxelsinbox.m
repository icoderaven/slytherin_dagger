function [mat,filledvoxels,flag,voxelsinbox]=findvoxelsinbox(voxels,point,d,steps)
r=d/2;

ind=find(voxels(1,:)<point(1)+r & voxels(1,:)>point(1)-r & voxels(2,:)<point(2)+r & voxels(2,:)>point(2)-r & voxels(3,:)<point(3)+r & voxels(3,:)>point(3)-r);
if isempty(ind)
%     flag=0;
flag=1;
    mat=0;
    filledvoxels=0;
    voxelsinbox = 0;
else
    
    voxelsinbox=voxels(:,ind);
    bottomleftpoint=bsxfun(@minus,point,[r;r;r]);
    validpoints=bsxfun(@minus,voxelsinbox,bottomleftpoint);
    flag=1;
    validpointsscaled=floor(validpoints*steps/d)+1;
    mat=zeros(steps,steps,steps);
    ind2=sub2ind(size(mat),validpointsscaled(1,:),validpointsscaled(2,:),validpointsscaled(3,:));
    mat(ind2)=1;
    filledvoxels=ind2;
end
end