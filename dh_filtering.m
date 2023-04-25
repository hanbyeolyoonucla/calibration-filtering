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

% calibrated DH from Kalibrot
calibratedDH = [0	0	0	135;
-1.567671326	-0.09418696603	-1.567522278	0.6327129301;
0.00232117105	134.99947	-0.001921290774	0;
-1.57539267	37.78467258	0.004128683254	119.8986164;
1.572228153	-0.04966838003	-0.0007294401206	-0.003682920773;
-1.567004188	0.0002605447373	3.141592654	70]; % alpha a theta d 

% calibrated DH from RoboDK
% calibratedDH = [0.0000000,-1.5685302,-0.0015844,-1.5710132,1.5713967,-1.5703714
% 0.0000000,0.3760858,134.9888038,37.8811929,0.0460869,0.0019857
% 0.0000000,-1.5714982,0.0019691,0.0000571,0.0018524,3.1415927
% 135.0000000,0.1216121,0.0000000,120.0658861,-0.0415388,70.0000000]';

% tcp information
TCP = [-0.5732614908	-0.1687166436	158.138543	170.0399912	85.77777461	9.550385364];
T_tool_rdk = XYZWPR2SE3(TCP);
% T_tool_rdk = [1	0	0	24.29335976
% 0	1	0	-9.705854952
% 0	0	1	54.81243107
% 0	0	0	1];

% DH following convention: theta,d,a,alpha
cdh = calibratedDH(:, [3 4 2 1]);
ndh = DH_nominal(:,[2 1 3 4]);
cq = cdh(:,1)'; % zero config
nq = ndh(:,1)'; % zero config

% define robot
for ii = 1:6
    cL(ii) = Revolute('d', cdh(ii,2), 'a', cdh(ii,3), 'alpha', cdh(ii,4),'offset', cdh(ii,1),'modified');
    nL(ii) = Revolute('d', ndh(ii,2), 'a', ndh(ii,3), 'alpha', ndh(ii,4),'offset', ndh(ii,1),'modified');
end
crobot = SerialLink(cL,'tool',T_tool_rdk,'name',"Final Robot");
nrobot = SerialLink(nL,'tool',T_tool_rdk,'name',"Final Robot");

% robot figure
% figure()
% title('Calibrated Robot')
% crobot.plot(cq)
% figure()
% title('Nominal Robot')
% nrobot.plot(nq)

%% 2. Filtering: calibrated IK and nominal FK

% pre-filtered trajectory
traj_prefilt = readmatrix('./data/typo_1.2F-4/Unfiltered_occ_01.txt'); % x y z Rx Ry Rz [mm,deg]
% traj_prefilt = [];
Ts_prefilt = XYZWPR2SE3(traj_prefilt);
N = size(traj_prefilt,1);

% initial guess
q0 = [4.562069,38.431293,17.880776,-94.485517,-100.693966,64.669828]*pi/180;
% q0 = cq + q0;

% for ii = 1:N
%     joint_filtered(ii,:) = crobot.ikine(Ts_prefilt(:,:,ii),'q0',q0);
%     disp(joint_filtered(ii,:))
% end

% filter trajectory: calibrated IK
joint_filtered = crobot.ikine(Ts_prefilt,'q0',q0);
% joint_filtered = joint_filtered - cq;
% joint_filtered = joint_filtered + [0 0 0 0 0 2*pi];

% filter trajectory: nominal FK
Ts_filtered = nrobot.fkine(joint_filtered);
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
writematrix(traj_filtered, './result/typo_1.2F-4/occ_01.txt','Delimiter','tab')
% writematrix(joint_filtered*180/pi, './result/axial_path_filtered_joint_angles_petercorke.txt')

%%
% figure()
% nrobot.plot(joint_filtered)
% SE32mat(crobot.fkine([4.562069,38.431293,17.880776,-94.485517,-100.693966,64.669828]*pi/180))