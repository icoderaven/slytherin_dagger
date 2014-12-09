function plot_dirlines(anchor_pt,normal_vec,d)
% it normalizes the normal_vec
counter = 1;
for j=1:length(anchor_pt)
    for i=1:size(anchor_pt{1},1)
        tmp_vec = normal_vec{j}(i,:)/norm(normal_vec{j}(i,:));
        tmp =repmat(anchor_pt{j}(i,:),[d(counter)+1,1])+repmat([0:d(counter)].',[1,3]).*repmat(tmp_vec,[d(counter)+1,1]);
        plot3(tmp(:,1),tmp(:,2),tmp(:,3),'r')
        counter = counter+1;
    end
end