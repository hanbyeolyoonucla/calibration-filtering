function T = XYZWPR2SE3(trajectory)
%TRAJECTORY2SE3 trajectory [x y z Rx Ry Rz] into SE3
%   Detailed explanation goes here

N = size(trajectory,1); % number of points in trajectory
for ii = 1:N
    temp = trajectory(ii,:);
    p = temp(1:3)';
    R = eul2rotm(temp(4:6)*pi/180,'XYZ');
    T(:,:,ii) = [R p; zeros(1,3) 1];    
end
% T = SE3(T)
end

