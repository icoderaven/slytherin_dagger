function [d] = distance2obstacle(pt,vec,voxels,step,maxdist) 
% normalizes vec;
pt = pt(:);
vec = vec(:);
num_steps = floor(maxdist/step);
d = maxdist;
for i=1:num_steps+1
    newp = pt + step*(i-1)*(vec/norm(vec));
    [~,~,flag,~]= findvoxelsinbox(voxels,newp,5,1);
    if flag==1 % if filled
        d = (i-1)*step;   
        break;
    end
end

end
% P1 = snakePoints(end,:);
%     P2 = snakePoints(end,:)+100*(snakePoints(end,:)-snakePoints(end-1,:))/norm((snakePoints(end,:)-snakePoints(end-1,:)));
%     pts = [P1; P2];
%     line(pts(:,1), pts(:,2), pts(:,3),'color','r')
%     
%   tmp = (snakePoints(end,:)-snakePoints(end-1,:))/norm((snakePoints(end,:)-snakePoints(end-1,:)));
%   [TH,R,Z] = cart2pol(tmp(1),tmp(2),tmp(3));
%   [tmp2(1),tmp2(2),tmp2(3)] = pol2cart(TH+pi/2,R,Z);
%   P2tmp = P1 + tmp2;
%   pts2 = [P1;P2tmp];
%   line(pts2(:,1), pts2(:,2), pts2(:,3),'color','g')
%   
%   [TH,R,Z] = cart2pol(tmp(1),tmp(2),tmp(3));
%   [tmp2(1),tmp2(2),tmp2(3)] = pol2cart(TH-pi/2,R,Z);
%   P2tmp = P1 + tmp2;
%   pts2 = [P1;P2tmp];
%   line(pts2(:,1), pts2(:,2), pts2(:,3),'color','m')
%   
%   tmp3 = pol2cart(TH,0,Z+10);
%   pts3 = [P1;P1+tmp3];
%   line(pts2(:,1), pts2(:,2), pts2(:,3),'color','k')
%   
%   line([P1(1);0.5*P1(1)+0.5*P2tmp(1)],[P1(2);0.5*P1(2)+0.5*P2tmp(2)],[P1(3);0.5*P1(3)+0.5*P2tmp(3)],'color','b')
