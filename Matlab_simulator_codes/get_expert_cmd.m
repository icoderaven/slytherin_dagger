function [yaw,pitch,state] = get_expert_cmd(state,yaw,pitch,maxrange,inc,val)   

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
        else
        display('press a different key');
    
    
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
        % add randomness?
        state=adderror(state,0);
    end