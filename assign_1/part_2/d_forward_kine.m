function [ dpos, drot ] = d_forward_kine( actu_ang )
% for finding the variation in end effector position with the change in
% actuation angle
global link_length;
num_of_links = length(link_length);
num_of_params = length(actu_ang);

%%
dpos = zeros(4,num_of_params);
drot = zeros(4,num_of_params);
for link_cons = 1:num_of_links
    T = eye(4);
    for i = 1:link_cons-1
        T = T * rotx( actu_ang( (i-1) * 3 + 1 ) ) * roty( actu_ang( (i-1) * 3 + 2 ) ) * rotz( actu_ang( (i-1) * 3 + 3 ) ) * trans(link_length(i));
    end
    
    i = link_cons;
    T_rotx = T * d_rotx( actu_ang( (i-1) * 3 + 1 ) ) * roty( actu_ang( (i-1) * 3 + 2 ) ) * rotz( actu_ang( (i-1) * 3 + 3 ) ) * trans(link_length(i));
    T_roty = T * rotx( actu_ang( (i-1) * 3 + 1 ) ) * d_roty( actu_ang( (i-1) * 3 + 2 ) ) * rotz( actu_ang( (i-1) * 3 + 3 ) ) * trans(link_length(i));
    T_rotz = T * rotx( actu_ang( (i-1) * 3 + 1 ) ) * roty( actu_ang( (i-1) * 3 + 2 ) ) * d_rotz( actu_ang( (i-1) * 3 + 3 ) ) * trans(link_length(i));
    
    for i = link_cons+1:num_of_links
        T_rotx = T_rotx*rotx( actu_ang( (i-1) * 3 + 1 ) ) * roty( actu_ang( (i-1) * 3 + 2 ) ) * rotz( actu_ang( (i-1) * 3 + 3 ) ) * trans(link_length(i));
        T_roty = T_roty*rotx( actu_ang( (i-1) * 3 + 1 ) ) * roty( actu_ang( (i-1) * 3 + 2 ) ) * rotz( actu_ang( (i-1) * 3 + 3 ) ) * trans(link_length(i));
        T_rotz = T_rotz*rotx( actu_ang( (i-1) * 3 + 1 ) ) * roty( actu_ang( (i-1) * 3 + 2 ) ) * rotz( actu_ang( (i-1) * 3 + 3 ) ) * trans(link_length(i));
    end
    
    %% change in X, Y, Z due to rot along X Y Z
    dpos(:,(link_cons-1)*3 + 1) = T_rotx(:,4);
    dpos(:,(link_cons-1)*3 + 2) = T_roty(:,4);
    dpos(:,(link_cons-1)*3 + 3) = T_rotz(:,4);
    
    %%
    T_rotx = T_rotx*trans(x_spear(1));
    T_roty = T_roty*trans(x_spear(1));
    T_rotz = T_rotz*trans(x_spear(1));
    
    drot(:,(link_cons-1)*3 + 1) = T_rotx(:,4);
    drot(:,(link_cons-1)*3 + 2) = T_roty(:,4);
    drot(:,(link_cons-1)*3 + 3) = T_rotz(:,4);
end



end

