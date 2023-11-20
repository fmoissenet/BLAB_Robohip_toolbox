% -------------------------------------------------------------------------
% INIT THE WORKSPACE
% -------------------------------------------------------------------------
clearvars;
close all;
warning off;
clc;

% -------------------------------------------------------------------------
% SET FOLDERS
% -------------------------------------------------------------------------
Folder.toolbox = 'C:\Users\moissene\OneDrive - unige.ch\2022 - ROBOHIP\Development\Biomécanique\BLAB_Robohip_toolbox\';
Folder.data    = 'C:\Users\moissene\OneDrive - unige.ch\2022 - ROBOHIP\Development\Biomécanique\calibrage_KUKA\';
Folder.dep     = [Folder.toolbox,'dependencies\'];
addpath(Folder.toolbox);
addpath(genpath(Folder.dep));
cd(Folder.data);

% -------------------------------------------------------------------------
% QTM PARAMETERS
% -------------------------------------------------------------------------
% Processing > 3D tracking > Prediction error = 0.5 mm
%                          > Maximum residual = 0.5 mm
%                          > Minimum trajectory length = 10 frames
% Processing > 6DOF Tracking > KUKA_flange > Virtual / Min. markers = 4
%                                          > Id / Max. residual = 0.5 mm
%                                          > Bone tolerance = 0.5 mm

%% ------------------------------------------------------------------------
% OPTIMISE ROBOT BASE POSE IN CAMERA FRAME
% -------------------------------------------------------------------------

% Load calibration file
load('calibration-2023-11-20-01.mat');
camPoint0  = calibration_data.camera_eef_pose(:,1:3)*1e3;
robPoint0  = calibration_data.robot_eef_pose(:,1:3)*1e3;

% Remove data with NaN (related to camera issues)
indPoint   = find(~isnan(camPoint0(:,1)));
camPoint0  = camPoint0(indPoint,:);
robPoint0  = robPoint0(indPoint,:);
clear indPoint;

% Store initial errors
error0.X   = sqrt((camPoint0(:,1)-robPoint0(:,1)).^2);
error0.Y   = sqrt((camPoint0(:,2)-robPoint0(:,2)).^2);
error0.Z   = sqrt((camPoint0(:,3)-robPoint0(:,3)).^2);
error0.e3D = sqrt(sum((camPoint0-robPoint0).^2,2));

% % Compute least square rigid transformation between camera and robot frame
[R,d,rms]  = soder(robPoint0,camPoint0); % y = R*x+d
temp       = rad2deg(R2mobileZYX_array3(R));
x1         = [[temp(:,3) temp(:,2) temp(:,1)],d']; % To be adapted to the selected Euler decomposition
camPoint1  = camPoint0;
robPoint1  = ((R*robPoint0')+d)';
error1.X   = sqrt((camPoint1(:,1)-robPoint1(:,1)).^2);
error1.Y   = sqrt((camPoint1(:,2)-robPoint1(:,2)).^2);
error1.Z   = sqrt((camPoint1(:,3)-robPoint1(:,3)).^2);
error1.e3D = sqrt(sum((camPoint1-robPoint1).^2,2));

% New camera frame in QTM
disp(['New camera frame in QTM: ',num2str(x1(1:6))]);
% These values are set in QTM as rigid transformation of the initial camera frame
% User must check that Euler angles in QTM are set as "Qualisys standard"

%% ------------------------------------------------------------------------
% TEST TRANSFORMATION WITH NEW POINTCLOUD
% -------------------------------------------------------------------------

% Load calibration file
load('calibration-2023-11-20-02.mat');
camPoint2  = calibration_data.camera_eef_pose(:,1:3)*1e3;
robPoint2  = calibration_data.robot_eef_pose(:,1:3)*1e3;

% Remove data with NaN (related to camera issues)
indPoint   = find(~isnan(camPoint2(:,1)));
camPoint2  = camPoint2(indPoint,:);
robPoint2  = robPoint2(indPoint,:);
clear indPoint;

% Store initial errors
error2.X   = sqrt((camPoint2(:,1)-robPoint2(:,1)).^2);
error2.Y   = sqrt((camPoint2(:,2)-robPoint2(:,2)).^2);
error2.Z   = sqrt((camPoint2(:,3)-robPoint2(:,3)).^2);
error2.e3D = sqrt(sum((camPoint2-robPoint2).^2,2));

%% ------------------------------------------------------------------------
% PLOT
% ------------------------------------------------------------------------- 
figure();
hold on;
axis equal;
plot3(robPoint0(:,1),robPoint0(:,2),robPoint0(:,3),'Marker','.','Markersize',15,'Color','black','LineStyle','none');
plot3(camPoint0(:,1),camPoint0(:,2),camPoint0(:,3),'Marker','o','Markersize',5,'Color','black','LineStyle','none');
plot3(robPoint1(:,1),robPoint1(:,2),robPoint1(:,3),'Marker','.','Markersize',15,'Color','red','LineStyle','none');
plot3(camPoint1(:,1),camPoint1(:,2),camPoint1(:,3),'Marker','o','Markersize',5,'Color','red','LineStyle','none');
plot3(robPoint2(:,1),robPoint2(:,2),robPoint2(:,3),'Marker','.','Markersize',15,'Color','green','LineStyle','none');
plot3(camPoint2(:,1),camPoint2(:,2),camPoint2(:,3),'Marker','o','Markersize',5,'Color','green','LineStyle','none');

figure();
subplot(1,4,1); hold on; plot(error1.X,'red'); plot(error2.X,'green');
subplot(1,4,2); hold on; plot(error1.Y,'red'); plot(error2.Y,'green');
subplot(1,4,3); hold on; plot(error1.Z,'red'); plot(error2.Z,'green');
subplot(1,4,4); hold on; plot(error1.e3D,'red'); plot(error2.e3D,'green');

%% ------------------------------------------------------------------------
% TEST
% ------------------------------------------------------------------------- 

% Set 3D position of the TRIPERON obtained from QTM
a1 = -0.1000;
a2 = -0.0600;
a3 = 0.0200;
R1 = [1 0 0; 0 cosd(a1) -sind(a1); 0 sind(a1) cosd(a1)];
R2 = [cosd(a2) 0 sind(a2); 0 1 0; -sind(a2) 0 cosd(a2)];
R3 = [cosd(a3) -sind(a3) 0; sind(a3) cosd(a3) 0; 0 0 1];
Rt = R3*R2*R1;
tt = [537.6600; -549.5800; -2.1600];

% Set rotation matrix obtained from Matlab
Rt2 = [1 -0.0003 -0.0010; 0.0003 1 0.0017; 0.0010 -0.0017 1];

% Get TRIPTERON markers
c3dFile = 'test.c3d';
btkFile = btkReadAcquisition(c3dFile);
Marker  = btkGetMarkers(btkFile);

% Plot
figure; hold on; axis equal;
plot3(mean(Marker.TRIPTERON_p2(:,1)),mean(Marker.TRIPTERON_p2(:,2)),mean(Marker.TRIPTERON_p2(:,3)),'Marker','.','Markersize',15,'Color','red');
plot3(mean(Marker.LinmotX2(:,1)),mean(Marker.LinmotX2(:,2)),mean(Marker.LinmotX2(:,3)),'Marker','.','Markersize',15,'Color','red');
plot3(mean(Marker.LinmotY2(:,1)),mean(Marker.LinmotY2(:,2)),mean(Marker.LinmotY2(:,3)),'Marker','.','Markersize',15,'Color','red');
plot3(mean(Marker.LinmotZ1(:,1)),mean(Marker.LinmotZ1(:,2)),mean(Marker.LinmotZ1(:,3)),'Marker','.','Markersize',15,'Color','red');
quiver3(tt(1),tt(2),tt(3),Rt(1,1),Rt(2,1),Rt(3,1),100,'red');
quiver3(tt(1),tt(2),tt(3),Rt(1,2),Rt(2,2),Rt(3,2),100,'green');
quiver3(tt(1),tt(2),tt(3),Rt(1,3),Rt(2,3),Rt(3,3),100,'blue');
quiver3(tt(1),tt(2),tt(3),Rt2(1,1),Rt2(2,1),Rt2(3,1),100,'--r');
quiver3(tt(1),tt(2),tt(3),Rt2(1,2),Rt2(2,2),Rt2(3,2),100,'--g');
quiver3(tt(1),tt(2),tt(3),Rt2(1,3),Rt2(2,3),Rt2(3,3),100,'--b');