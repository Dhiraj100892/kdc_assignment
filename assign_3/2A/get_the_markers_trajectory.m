function [ markers_traj ] = get_the_markers_trajectory(fname )
% input is the data file from which we want to get the markers position
[D,names,units,freq] = mrdplot_convert(fname);
markers_traj = zeros(size(D,1),8,3);
num_markers = 8 ;
for i = 1:size(D,1)
    start_indx = findMRDPLOTindex(names,'ml0x');
    for j = 1:num_markers
       for k = 1:3
           markers_traj(i,j,k) = D(i,start_indx);
           start_indx = start_indx + 1;
       end
    end
end
end