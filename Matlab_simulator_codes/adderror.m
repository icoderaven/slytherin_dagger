function state=adderror(state,flag)
    len=length(state)-6;
    if flag==1
    for ii=1:len
        state(ii+6)=state(ii+6)+rand*pi/180;
    end
    else
    for ii=1:len
        state(ii+6)=state(ii+6)+rand*0.2*pi/180;
    end
    end    
end