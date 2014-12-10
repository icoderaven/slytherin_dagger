function fv=scale_stl(fv,R,scale,offset)
coords=R*[fv.vertices(:,1)';fv.vertices(:,2)';fv.vertices(:,3)'];
x=coords(1,:)/scale+offset(1);
y=coords(2,:)/scale+offset(2);
z=coords(3,:)/scale+offset(3);
fv.vertices=[x',y',z'];


end