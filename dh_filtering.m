clc; clear; close all;
addpath('./function');

% 2023-03-29 HBY
% filter trajectory based on calibrated DH
% 1. given desired trajectory and nominal & calibrated DH
% 2. solve calibrated IK numerically -> get filtered joint trajectory
% 3. solve nominal FK -> get filtered trajectory

%% 1. Define Nominal & Calibrated Robot

% nominal DH
DH_nominal = ...
    [135 0 0 0 0;
    0 -pi/2 0 -pi/2 0;
    0 0 135 0 0;
    120 0 38 -pi/2 0;
    0 0 0 pi/2 0;
    70 pi 0 -pi/2 0]; % d theta a alpha

% calibrated DH from RoboDK
calibratedDH = [0.0000000,-1.5685302,-0.0015844,-1.5710132,1.5713967,-1.5703714;
0.0000000,0.3760858,134.9888038,37.8811929,0.0460869,0.0019857;
0.0000000,-1.5714982,0.0019691,0.0000571,0.0018524,3.1415927;
135.0000000,0.1216121,0.0000000,120.0658861,-0.0415388,70.0000000]'; % alpha a theta d 

% tcp information
TCP = [-0.4590164281	-0.9164464634	161.3819234	168.2432879	85.64040621	11.58103583];
T_tool_rdk = XYZWPR2SE3(TCP);
% T_tool_rdk = eye(4,4);

% DH following convention: theta,d,a,alpha
cdh = calibratedDH(:, [3 4 2 1]);
ndh = DH_nominal(:,[2 1 3 4]);
cq = cdh(:,1)'; % zero config
nq = ndh(:,1)'; % zero config

% define robot
for ii = 1:6
    cL(ii) = Revolute('d', cdh(ii,2), 'a', cdh(ii,3), 'alpha', cdh(ii,4),'modified');
    nL(ii) = Revolute('d', ndh(ii,2), 'a', ndh(ii,3), 'alpha', ndh(ii,4),'modified');
end
crobot = SerialLink(cL,'tool',T_tool_rdk,'name',"Final Robot");
nrobot = SerialLink(nL,'tool',T_tool_rdk,'name',"Final Robot");

% robot figure
figure()
title('Calibrated Robot')
crobot.plot(cq)
figure()
title('Nominal Robot')
nrobot.plot(nq)

%% 2. Filtering: calibrated IK and nominal FK

% pre-filtered trajectory
traj_prefilt = readmatrix('./data/typo_1.2A-7/Unfiltered_occ_01.txt'); % x y z Rx Ry Rz [mm,deg]
Ts_prefilt = XYZWPR2SE3(traj_prefilt);
N = size(traj_prefilt,1);

% initial guess
q0 = [9.12000000000000	-11.5900000000000	48.5200000000000	-84.7500000000000	-82.7600000000000	97.7500000000000]*pi/180;
q0 = cq + q0;

% filter trajectory: calibrated IK
joint_filtered = crobot.ikine(Ts_prefilt,'q0',q0);
joint_filtered = joint_filtered - cq;
joint_filtered = joint_filtered + [0 0 0 0 0 2*pi];

% filter trajectory: nominal FK
Ts_filtered = nrobot.fkine(nq + joint_filtered);
p_filtered = transl(Ts_filtered);

for ii = 1:length(Ts_filtered)
    R_filtered(:,:,ii) = t2r(Ts_filtered(ii));
    eul_filtered(ii,:) =  rotm2eul(R_filtered(:,:,ii), 'XYZ');
    eul_filtered(ii,:) = eul_filtered(ii,:)*180/pi;
end
traj_filtered = [p_filtered eul_filtered];

%% 3. Check Result

% % robodk filtered traj
% rdk_traj = readmatrix('./data/axial_path_filtered_robodk.txt');

% plot result
figure()
plot3(traj_prefilt(:,1),traj_prefilt(:,2),traj_prefilt(:,3),'.'); % unfiltered
hold on
grid on
plot3(p_filtered(:,1),p_filtered(:,2),p_filtered(:,3),'.'); % peter corke filtered
legend('prefiltered','filtered')
xlabel('x')
ylabel('y')
zlabel('z')
axis equal

% save the result
writematrix(traj_filtered, './result/typo_1.2A-7/occ_01_petercorke.txt','Delimiter','tab')
% writematrix(joint_filtered*180/pi, './result/axial_path_filtered_joint_angles_petercorke.txt')

