function [ F,fv2] = findFilledVoxelsAndRender(name,R,scale,offset )
fv2 = stlread(name);
fv2=scale_stl(fv2,R,scale*2,offset);
[OUTPUTgrid2] = VOXELISE(20,20,20,name,'xyz');
[x2,y2,z2]=ind2sub(size(OUTPUTgrid2), find(OUTPUTgrid2));
voxels2=R*[x2';y2';z2']/scale;
x2=voxels2(1,:)+offset(1);
y2=voxels2(2,:)+offset(2);
z2=voxels2(3,:)+offset(3);
F=[x2',y2',z2'];

end

