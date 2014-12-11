function plot_dirlines(anchor_pt,normal_vec,d)
% it normalizes the normal_vec
for j=1:length(anchor_pt)
    for i=1:size(anchor_pt{1},1)
        tmp_vec = normal_vec{j}(i,:)/norm(normal_vec{j}(i,:));
        tmp =repmat(anchor_pt{j}(i,:),[d(j,i)+1,1])+repmat([0:d(j,i)].',[1,3]).*repmat(tmp_vec,[d(j,i)+1,1]);
        if d(j,i) >45
            plot3(tmp(:,1),tmp(:,2),tmp(:,3),'b')
        elseif d(j,i)>20
            plot3(tmp(:,1),tmp(:,2),tmp(:,3),'g','LineWidth',3)
        else
            plot3(tmp(:,1),tmp(:,2),tmp(:,3),'r','LineWidth',3)
        end
    end
end