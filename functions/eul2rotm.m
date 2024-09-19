function rotm = eul2rotm(eul)
%% Implementation of eul2rotm without the Robotics System Toolbox
% Rotation order: X->Y->Z
% Author: Taro Suzuki

n = size(eul,1);
c = cos(eul);
s = sin(eul);

rotm = zeros(3,3,n);
for i=1:n
    Rx = [1 0 0; 0 c(i,1) -s(i,1); 0 s(i,1) c(i,1)];
    Ry = [c(i,2) 0 s(i,2); 0 1 0; -s(i,2) 0 c(i,2)];
    Rz = [c(i,3) -s(i,3) 0; s(i,3) c(i,3) 0; 0 0 1];
    
    rotm(:,:,i) = Rx*Ry*Rz;
end