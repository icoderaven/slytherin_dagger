%hold on
close all
clear
clc
P3=[40,0,0];

pitch0=0;
yaw0=0;
[phi0,theta0]=pithyawtoaxisangle(pitch0,yaw0);
state=[0,0,0,0,0,0,phi0,theta0];
drawColor=[0.2 length(state)/66 0.3 ];%drawColor=drawColor/norm(drawColor);
LINK_LENGTH=4;
LINK_RADIUS=1.5;
drawType=1;
Tregister=eye(4);
linkStartDraw=0;
%axis([0 150 -50 50 -50 50]);
%Voxelise the STL:
R=[1 0 0; 0 0 -1 ; 0  1 0];

[OUTPUTgrid] = VOXELISE(20,20,20,'heart.STL','xyz');
[x,y,z]=ind2sub(size(OUTPUTgrid), find(OUTPUTgrid));
voxels=R*[x';y';z'];
x=voxels(1,:)+10;
y=voxels(2,:)+5;
z=voxels(3,:)-10;
figure('units','normalized','outerposition',[0 0 1 1])
axis([0 150 -50 50 -50 50]);
grid on
hold on
subplot(1,2,1)

scatter3(x,y,z,'red','s');
hold on
scatter3(P3(1),P3(2),P3(3),150,'green','fill')

[h, snakePoints] = drawState(state,drawColor,LINK_LENGTH,LINK_RADIUS,drawType,Tregister,linkStartDraw);
subplot(1,2,2)
scatter3(x,y,z);
[h, snakePoints] = drawState(state,drawColor,LINK_LENGTH,LINK_RADIUS,drawType,Tregister,linkStartDraw);
view([90+state(end-1) state(end)])
hold off
over=0;
%%
pitch=0;
yaw=0;
inc=1*pi/180;
maxrange=10*pi/180;
boxsize=20;
steps=10;
count=0;
while over==0 && length(state)<66
    
    %read from keyboard
    [voxel_mat,filled_voxels,flag]=findvoxelsinbox(voxels,snakePoints(end,:)',boxsize,steps);
    
    
    val=getkey();
    %display(val);
    if val ~=32 && val~=113
        
        if val==29
            pitch=pitch+inc;
        elseif val==28
            pitch=pitch-inc;
        elseif val==30
            yaw=yaw+inc;
        elseif val==31
            yaw=yaw-inc;
        end
        if pitch>maxrange
            pitch=maxrange;
        elseif pitch<-maxrange
            pitch=-maxrange;
        end
        
        if yaw>maxrange
            yaw=maxrange;
        elseif yaw<-maxrange
            yaw=-maxrange;
        end
        
        [phi,theta]=pithyawtoaxisangle(pitch,yaw);
        %display([pitch,yaw,phi,theta]);
        state(end-1)=phi;
        state(end)=theta;
        state=adderror(state,0);
    end
    if val==32
        if flag==1
            count=count+1;
            log_data{count}=[filled_voxels,pitch,yaw];
        end
        pitch=0;yaw=0;
        state=adderror(state,1);
        state=[state,0,0];
        
    end
    if val==113
        over=1;
    end
    clf
    
    
    drawColor=[0.2 length(state)/66 0.3 ];
    hold on
    subplot(2,2,1)
    scatter3(x,y,z,'red','fill','s');
    hold on
    scatter3(P3(1),P3(2),P3(3),150,'green','fill')
    [~, snakePoints] = drawState(state,drawColor,LINK_LENGTH,LINK_RADIUS,drawType,Tregister,linkStartDraw);
    P1 = snakePoints(end,:);
    P2 = snakePoints(end,:)+100*(snakePoints(end,:)-snakePoints(end-1,:))/norm((snakePoints(end,:)-snakePoints(end-1,:)));
    pts = [P1; P2];
    line(pts(:,1), pts(:,2), pts(:,3))
    axis([0 150 -50 50 -50 50]);
    view(3)
    
    subplot(2,2,2)
    scatter3(x,y,z,'red','fill','s');
    hold on
    scatter3(P3(1),P3(2),P3(3),150,'green','fill')
    [~, snakePoints] = drawState(state,drawColor,LINK_LENGTH,LINK_RADIUS,drawType,Tregister,linkStartDraw);
    line(pts(:,1), pts(:,2), pts(:,3))
    axis([0 150 -50 50 -50 50]);
    %view([90+state(end-1)*180/pi state(end)*180/pi])
    view([0,0])
    
    
    subplot(2,2,3)
    scatter3(x,y,z,'red','fill','s');
    hold on
    scatter3(P3(1),P3(2),P3(3),150,'green','fill')
    [~, snakePoints] = drawState(state,drawColor,LINK_LENGTH,LINK_RADIUS,drawType,Tregister,linkStartDraw);
    line(pts(:,1), pts(:,2), pts(:,3))
    axis([0 150 -50 50 -50 50]);
    %view([90+state(end-1)*180/pi state(end)*180/pi])
    view([90,0])
    
    
    
    subplot(2,2,4)
    scatter3(x,y,z,'red','fill','s');
    hold on
    scatter3(P3(1),P3(2),P3(3),150,'green','fill')
    [~, snakePoints] = drawState(state,drawColor,LINK_LENGTH,LINK_RADIUS,drawType,Tregister,linkStartDraw);
    line(pts(:,1), pts(:,2), pts(:,3))
    axis([0 150 -50 50 -50 50]);
    %view([90+state(end-1)*180/pi state(end)*180/pi])
    view([0,90])
    
    
    hold off
    
    
    
end
h=datestr(clock,30);
save(h,'log_data')
% end