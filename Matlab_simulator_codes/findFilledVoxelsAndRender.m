function [ F,fv] = findFilledVoxelsAndRender(name,R,scale,offset )
fv = stlread(name);
fv=scale_stl(fv,R,scale,offset);

%OUTPUTgrid=VOXELISE(20,20,20,fv);
%[OUTPUTgrid2] = VOXELISE(20,20,20,name,'xyz');
%[x,y,z]=ind2sub(size(OUTPUTgrid), find(OUTPUTgrid));
%voxels=R*[x';y';z'];
%x=voxels(1,:)/scale+offset(1);
%y=voxels(2,:)/scale+offset(2);
%z=voxels(3,:)/scale+offset(3);
F=[0,0,0];

end

