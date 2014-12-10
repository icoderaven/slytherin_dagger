function [x,y,z] = scale_shift(voxels,fv)
% normalize voxels and make start at 0;
x = (voxels(1,:) - min(voxels(1,:)));
x = x/max(x);
y = (voxels(2,:) - min(voxels(2,:)));
y = y/max(y);
z = (voxels(3,:) - min(voxels(3,:)));
z = z/max(z);

%scale by the size of the vertices and shift by the min of vertices
tmp1 = sort(fv.vertices(:,1));
x= x*(tmp1(end)-tmp1(1)) + tmp1(1);
tmp1 = sort(fv.vertices(:,2));
y=y*(tmp1(end)-tmp1(1)) + tmp1(1);
tmp1 = sort(fv.vertices(:,3));
z=z*(tmp1(end)-tmp1(1)) + tmp1(1);

% 
% vox_h=[x',y',z'];
