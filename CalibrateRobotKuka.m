% -------------------------------------------------------------------------
% INIT THE WORKSPACE
% -------------------------------------------------------------------------
clearvars;
% close all;
warning off;
clc;

% -------------------------------------------------------------------------
% SET FOLDERS
% -------------------------------------------------------------------------
Folder.toolbox = 'C:\Users\moissene\OneDrive - unige.ch\2022 - ROBOHIP\Développement\Biomécanique\BLAB_Robohip_toolbox\';
Folder.data    = 'C:\Users\moissene\OneDrive - unige.ch\2022 - ROBOHIP\Développement\Biomécanique\calibrage_KUKA\';
Folder.dep     = [Folder.toolbox,'dependencies\'];
addpath(Folder.toolbox);
addpath(genpath(Folder.dep));
cd(Folder.data);

%% ------------------------------------------------------------------------
% OPTIMISE ROBOT BASE POSE IN CAMERA FRAME
% We need first to correctly define the robot base frame before to assess
% the flange
% -------------------------------------------------------------------------

% Load calibration file
load('calibration-2023-06-07-09-12-15-512.mat');
trobJoint = calibration_data.robot_joints;
tcamPoint = calibration_data.camera_eef_pose(:,1:3)*1e3;

% Set initial guess parameters (RT transformation and DH parameters)
x0 = [0 0 0,...                                       % Angle around X, Y, and Z
      0 0 0,...                                       % Displacement along X, Y, and Z
      [0.36,0.0,0.42,0.0,0.4,0.0,0.126+0.026]*1e3,... % d
      [0.0,0.0,0.0,0.0,0.0,0.0,0.0]*1e3,...           % a
      0,0,0,0,0,0,0,...                               % theta
      0,-pi/2,pi/2,pi/2,-pi/2,-pi/2,pi/2];            % alpha

% Remove data with NaN (related to camera issues)
robJoint = [];
camPoint = [];
for ipoint = 1:size(tcamPoint,1)
    if ~isnan(tcamPoint(ipoint,1))
        robJoint = [robJoint; trobJoint(ipoint,:)];
        camPoint = [camPoint; tcamPoint(ipoint,:)];
    end
end
robPoint = directKinematics(x0(7:end),x0(7:end),robJoint);

% Compute least square rigid transformation between camera and robot frame
options   = optimoptions(@fminunc,'PlotFcns',@optimplotfval,'Algorithm','quasi-newton','HessianApproximation','lbfgs','StepTolerance',1e-10,'MaxFunctionEvaluations',1e10,'MaxIterations',1e10);
x1        = fminunc(@(x1)costFunction1(x1,x0,camPoint,robJoint),x0,options);
RX        = [1 0 0; 0 cosd(x1(1)) -sind(x1(1)); 0 sind(x1(1)) cosd(x1(1))];
RY        = [cosd(x1(2)) 0 sind(x1(2)); 0 1 0; -sind(x1(2)) 0 cosd(x1(2))];
RZ        = [cosd(x1(3)) -sind(x1(3)) 0; sind(x1(3)) cosd(x1(3)) 0; 0 0 1];
R         = RZ*RY*RX;
d         = [x1(4) x1(5) x1(6)]';
camPoint1 = (inv(R)*(camPoint'-d))';
robPoint1 = robPoint;

% Store initial values
robPoint0 = robPoint;
camPoint0 = camPoint;

% New camera frame in QTM
disp(['New camera frame in QTM: ',num2str(x1(1:6))]);
% These values are set in QTM as rigid transformation of the initial camera frame

%% ------------------------------------------------------------------------
% OPTIMISE FLANGE POSE IN ROBOT BASE FRAME
% -------------------------------------------------------------------------

clearvars -except x1 camPoint0 robPoint0 camPoint1 robPoint1

% Load calibration file
load('calibration-32points_step2.mat');
n = size(calibration_data.camera_eef_pose,1);
trobJoint = calibration_data.robot_joints;
tcamPoint = calibration_data.camera_eef_pose(:,1:3)*1e3;

% Set initial guess parameters (RT transformation and DH parameters)
x0 = [0 0 0,...                                       % Angle around X, Y, and Z
      0 0 0,...                                       % Displacement along X, Y, and Z
      [0.36,0.0,0.42,0.0,0.4,0.0,0.126+0.026]*1e3,... % d
      [0.0,0.0,0.0,0.0,0.0,0.0,0.0]*1e3,...           % a
      0,0,0,0,0,0,0,...                               % theta
      0,-pi/2,pi/2,pi/2,-pi/2,-pi/2,pi/2];            % alpha

% Remove data with NaN (related to camera issues)
robJoint = [];
camPoint = [];
for ipoint = 1:n
    if ~isnan(tcamPoint(ipoint,1))
        robJoint = [robJoint; trobJoint(ipoint,:)];
        camPoint = [camPoint; tcamPoint(ipoint,:)];
    end
end
robPoint = directKinematics(x0(7:end),x0(7:end),robJoint);
n = size(robJoint,1);

% Remove outliers
indNotOutliers = find(sqrt(sum((camPoint-robPoint).^2,2))<5);
robJoint       = robJoint(indNotOutliers,:);
robPoint       = robPoint(indNotOutliers,:);
camPoint       = camPoint(indNotOutliers,:);
n = size(robJoint,1);

% Compute rigid transformation between flange and robot frame
[~,Tflange] = directKinematics(x0(7:end),x0(7:end),robJoint);

% Express camPoints in robotic flange frame
for ipoint = 1:n    
    camPoint(ipoint,:) = (inv(Tflange(1:3,1:3,ipoint))*(camPoint(ipoint,:)'-Tflange(1:3,4,ipoint)))';
    clear temp;
end

% Compute least square rigid transformation between camera and robot frame
options   = optimoptions(@fminunc,'PlotFcns',@optimplotfval,'Algorithm','quasi-newton','HessianApproximation','lbfgs','StepTolerance',1e-10,'MaxFunctionEvaluations',1e10,'MaxIterations',1e10);
x2        = fminunc(@(x2)costFunction2(x2,x0,camPoint,robJoint),x0,options);
RX        = [1 0 0; 0 cosd(x2(1)) -sind(x2(1)); 0 sind(x2(1)) cosd(x2(1))];
RY        = [cosd(x2(2)) 0 sind(x2(2)); 0 1 0; -sind(x2(2)) 0 cosd(x2(2))];
RZ        = [cosd(x2(3)) -sind(x2(3)) 0; sind(x2(3)) cosd(x2(3)) 0; 0 0 1];
R         = RZ*RY*RX;
d         = [x2(4) x2(5) x2(6)]';
camPoint2 = (inv(R)*(camPoint'-d))';
robPoint2 = robPoint;

% Express camPoints in robot frame
for ipoint = 1:n    
    camPoint2(ipoint,:) = ((Tflange(1:3,1:3,ipoint))*camPoint2(ipoint,:)'+Tflange(1:3,4,ipoint))';
    clear temp;
end

% New flange frame in QTM
disp(['New flange frame in QTM: ',num2str(x2(1:6))]);
% These values are set in QTM to apply the related rigid transformation of
% the flange 6DOF frame

%% ------------------------------------------------------------------------
% OPTIMISE DENAVIT HARTENBERG ROBOT PARAMETERS
% -------------------------------------------------------------------------

% Compute optimal DH parameters
x3        = fminunc(@(x3)costFunction3(x3,x0,camPoint2,robJoint),x0,options);
robPoint3 = directKinematics(x3(7:end),x0(7:end),robJoint);
camPoint3 = camPoint2;

% New DH parameters
disp(['Differences to apply in DH parameters: ',num2str(x3(7:end)*1e-3-x0(7:end)*1e-3)]);
% The values are applied in Matlab robot controller to update DH parameters

%% ------------------------------------------------------------------------
% LOAD FINAL DATA
% New pointset !!
% -------------------------------------------------------------------------

clearvars -except x1 x2 x3 camPoint0 robPoint0 camPoint1 robPoint1 camPoint2 robPoint2 camPoint3 robPoint3

% Load calibration file
load('calibration_14points_step3b.mat');
n = size(calibration_data.camera_eef_pose,1);
trobJoint = calibration_data.robot_joints;
tcamPoint = calibration_data.camera_eef_pose(:,1:3)*1e3;

% Set initial guess parameters (RT transformation and DH parameters)
x0 = x3;

% Remove data with NaN (related to camera issues)
robJoint = [];
camPoint = [];
for ipoint = 1:n
    if ~isnan(tcamPoint(ipoint,1))
        robJoint = [robJoint; trobJoint(ipoint,:)];
        camPoint = [camPoint; tcamPoint(ipoint,:)];
    end
end
robPoint = directKinematics(x0(7:end),x0(7:end),robJoint);
n = size(robJoint,1);

% Remove outliers
indNotOutliers = find(sqrt(sum((camPoint-robPoint).^2,2))<5);
robPoint4      = robPoint(indNotOutliers,:);
camPoint4      = camPoint(indNotOutliers,:);
n = size(robJoint,1);

%% ------------------------------------------------------------------------
% RESULTS
% -------------------------------------------------------------------------

% Store related errors
error0.X   = sqrt((camPoint0(:,1)-robPoint0(:,1)).^2);
error0.Y   = sqrt((camPoint0(:,2)-robPoint0(:,2)).^2);
error0.Z   = sqrt((camPoint0(:,3)-robPoint0(:,3)).^2);
error0.e3D = sqrt(sum((camPoint0-robPoint0).^2,2));
error1.X   = sqrt((camPoint1(:,1)-robPoint1(:,1)).^2);
error1.Y   = sqrt((camPoint1(:,2)-robPoint1(:,2)).^2);
error1.Z   = sqrt((camPoint1(:,3)-robPoint1(:,3)).^2);
error1.e3D = sqrt(sum((camPoint1-robPoint1).^2,2));
error2.X   = sqrt((camPoint2(:,1)-robPoint2(:,1)).^2);
error2.Y   = sqrt((camPoint2(:,2)-robPoint2(:,2)).^2);
error2.Z   = sqrt((camPoint2(:,3)-robPoint2(:,3)).^2);
error2.e3D = sqrt(sum((camPoint2-robPoint2).^2,2));
error3.X   = sqrt((camPoint3(:,1)-robPoint3(:,1)).^2);
error3.Y   = sqrt((camPoint3(:,2)-robPoint3(:,2)).^2);
error3.Z   = sqrt((camPoint3(:,3)-robPoint3(:,3)).^2);
error3.e3D = sqrt(sum((camPoint3-robPoint3).^2,2));
error4.X   = sqrt((camPoint4(:,1)-robPoint4(:,1)).^2);
error4.Y   = sqrt((camPoint4(:,2)-robPoint4(:,2)).^2);
error4.Z   = sqrt((camPoint4(:,3)-robPoint4(:,3)).^2);
error4.e3D = sqrt(sum((camPoint4-robPoint4).^2,2));

% Plot (FOR TEST ONLY)
figure;
subplot(4,1,1); axis equal; hold on; xlim([-inf inf]); ylim([0 5]); title('Error /X');
plot(error0.X,'red'); plot(error1.X,'--m'); plot(error2.X,'--b'); plot(error3.X,'--g'); plot(error4.X,'black');
subplot(4,1,2); axis equal; hold on; xlim([-inf inf]); ylim([0 5]); title('Error /Y');
plot(error0.Y,'red'); plot(error1.Y,'--m'); plot(error2.Y,'--b'); plot(error3.Y,'--g'); plot(error4.Y,'black');
subplot(4,1,3); axis equal; hold on; xlim([-inf inf]); ylim([0 5]); title('Error /Z');
plot(error0.Z,'red'); plot(error1.Z,'--m'); plot(error2.Z,'--b'); plot(error3.Z,'--g'); plot(error4.Z,'black');
subplot(4,1,4); axis equal; hold on; xlim([-inf inf]); ylim([0 5]); title('Error 3D');
plot(error0.e3D,'red'); plot(error1.e3D,'--m'); plot(error2.e3D,'--b'); plot(error3.e3D,'--g'); plot(error4.e3D,'black');

% Plot (FOR TEST ONLY)
figure; axis equal; hold on;
plot3(camPoint0(:,1),camPoint0(:,2),camPoint0(:,3),'Marker','o','Markersize',5,'Color','red','LineStyle','none');
plot3(robPoint0(:,1),robPoint0(:,2),robPoint0(:,3),'Marker','.','Markersize',15,'Color','red','LineStyle','none');
plot3(camPoint1(:,1),camPoint1(:,2),camPoint1(:,3),'Marker','o','Markersize',5,'Color','magenta','LineStyle','none');
plot3(robPoint1(:,1),robPoint1(:,2),robPoint1(:,3),'Marker','.','Markersize',15,'Color','magenta','LineStyle','none');
plot3(camPoint2(:,1),camPoint2(:,2),camPoint2(:,3),'Marker','o','Markersize',5,'Color','blue','LineStyle','none');
plot3(robPoint2(:,1),robPoint2(:,2),robPoint2(:,3),'Marker','.','Markersize',15,'Color','blue','LineStyle','none');
plot3(camPoint3(:,1),camPoint3(:,2),camPoint3(:,3),'Marker','o','Markersize',5,'Color','green','LineStyle','none');
plot3(robPoint3(:,1),robPoint3(:,2),robPoint3(:,3),'Marker','.','Markersize',15,'Color','green','LineStyle','none');
plot3(camPoint4(:,1),camPoint4(:,2),camPoint4(:,3),'Marker','o','Markersize',5,'Color','black','LineStyle','none');
plot3(robPoint4(:,1),robPoint4(:,2),robPoint4(:,3),'Marker','.','Markersize',15,'Color','black','LineStyle','none');

%% ------------------------------------------------------------------------
% RELATED FUNCTIONS
% -------------------------------------------------------------------------
function f = costFunction1(x,x0,camPoint,robJoint)
    RX        = [1 0 0; 0 cosd(x(1)) -sind(x(1)); 0 sind(x(1)) cosd(x(1))];
    RY        = [cosd(x(2)) 0 sind(x(2)); 0 1 0; -sind(x(2)) 0 cosd(x(2))];
    RZ        = [cosd(x(3)) -sind(x(3)) 0; sind(x(3)) cosd(x(3)) 0; 0 0 1];
    camPoint2 = (inv(RZ*RY*RX)*(camPoint'-[x(4) x(5) x(6)]'))';
    robPoint  = directKinematics(x0(7:end),x0(7:end),robJoint);
    f         = (sum(sqrt(sum((camPoint2-robPoint).^2,2)))/length(robPoint)).^2;
end

function f = costFunction2(x,x0,camPoint,robJoint)
    RX        = [1 0 0; 0 cosd(x(1)) -sind(x(1)); 0 sind(x(1)) cosd(x(1))];
    RY        = [cosd(x(2)) 0 sind(x(2)); 0 1 0; -sind(x(2)) 0 cosd(x(2))];
    RZ        = [cosd(x(3)) -sind(x(3)) 0; sind(x(3)) cosd(x(3)) 0; 0 0 1];
    camPoint2 = (inv(RZ*RY*RX)*(camPoint'-[x(4) x(5) x(6)]'))';
    f         = (sum(sqrt(sum((camPoint2).^2,2)))/length(camPoint2)).^2;
end

function f = costFunction3(x,x0,camPoint2,robJoint)
    robPoint2 = directKinematics(x(7:end),x0(7:end),robJoint);
    f         = (sum(sqrt(sum((camPoint2-robPoint2).^2,2)))/length(robPoint2)).^2;
end

function [robPoint,T7] = directKinematics(x,x0,robJoint)
    % Set robot parameters
    d     = x(1:7); % Only segment length are optimised (other kept at initial value x0)
    a     = x0(8:14);
    theta = x0(15:21);
    alfa  = x0(22:28);
    for t = 1:size(robJoint,1)
        q = robJoint(t,:);
        % Compute direct kinematics
        T = zeros(4,4,7);
        i = 1;
        T(:,:,i) = getDHMatrix(alfa(i),q(i)+theta(i),d(i),a(i));
        for i = 2:7
            T(:,:,i) = T(:,:,i-1)*getDHMatrix(alfa(i),q(i)+theta(i),d(i),a(i));
            T(:,:,i) = normalizeColumns(T(:,:,i));
        end
        robPoint(t,1:3) = T(1:3,4,7)';
        T7(:,:,t)       = T(:,:,7);
    end
end

function T = getDHMatrix(alfa,theta,d,a)
    T        = zeros(4,4);    
    calpha   = cos(alfa);
    sinalpha = sin(alfa);
    coshteta = cos(theta);
    sintheta = sin(theta);    
    T(1,1)   = coshteta;
    T(2,1)   = sintheta*calpha;
    T(3,1)   = sintheta*sinalpha;    
    T(1,2)   = -sintheta;
    T(2,2)   = coshteta*calpha;
    T(3,2)   = coshteta*sinalpha;   
    T(2,3)   = -sinalpha;
    T(3,3)   = calpha;    
    T(1,4)   = a;
    T(2,4)   = -sinalpha*d;
    T(3,4)   = calpha*d;
    T(4,4)   = 1;
end

function normalizedT = normalizeColumns(T)
    %% This function is used to normalize the columns of a rotation matrix with
    % some numerical errors resulting from matrix multiplication problems
    r = zeros(4,3); % corrected rotation matrix, with zero badding row at the end
    for j = 1:3
        r(1:3,j) = T(1:3,j)/norm(T(1:3,j));
    end
    normalizedT = [r,T(:,4)];
end

% 
% %% ------------------------------------------------------------------------
% % RELIABILITY / TEST 1 / 5x detection of the static robot base
% % -------------------------------------------------------------------------
% clearvars -except Folder reliability;
% 
% % Base position and orientation /camera
% cdata_c = nan(19,3,5);
% camFiles = dir('Calib_19points_trial*.mat');
% for ifile = 1:size(camFiles,1)
%     camFile  = camFiles(ifile).name;
%     temp     = load(camFile);
%     fileName = fieldnames(temp);
%     data     = temp.(fileName{1});
%     eventInd = [];
%     for ievent = 1:size(data.Events,1)
%         if ievent == 2
%             eventInd = [eventInd NaN]; % Position 2 not seen
%             eventInd = [eventInd data.Events(ievent).Frame];
%         elseif ievent == 4
%             eventInd = [eventInd NaN]; % Position 5 not seen
%             eventInd = [eventInd data.Events(ievent).Frame];
%         else
%             eventInd = [eventInd data.Events(ievent).Frame];
%         end
%     end
%     eventInd = fix(eventInd);
%     for ievent = 1:length(eventInd)
%         if ~isnan(eventInd(ievent))
%             cdata_cp(ievent,:,ifile) = permute(data.RigidBodies.Positions(1,:,eventInd(ievent)),[3,2,1]);
%             temp_cr                  = permute(data.RigidBodies.Rotations(1,:,eventInd(ievent)),[3,2,1]);
%             R_cr                     = [[temp_cr(1) temp_cr(2) temp_cr(3)]',[temp_cr(4) temp_cr(5) temp_cr(6)]',[temp_cr(7) temp_cr(8) temp_cr(9)]'];
%             Euler                    = rad2deg(R2mobileXYZ_array3(R_cr));
%             cdata_cr(ievent,:,ifile) = Euler;
%         else
%             cdata_cp(ievent,:,ifile) = NaN(1,3);
%             cdata_cr(ievent,:,ifile) = NaN(1,3);
%         end
%     end
%     clear camFile temp eventInd data fileName ievent temp_cr;
% end
% 
% % Assess reliability as standard deviation around the mean value
% % /position
% reliability.test1.camera.X.values = [];
% reliability.test1.camera.Y.values = [];
% reliability.test1.camera.Z.values = [];
% for ipoint = 1:19
%     for itrial = 1:5
%         reliability.test1.camera.X.values = [reliability.test1.camera.X.values cdata_cp(ipoint,1,itrial)-nanmean(cdata_cp(ipoint,1,:))];
%         reliability.test1.camera.Y.values = [reliability.test1.camera.Y.values cdata_cp(ipoint,2,itrial)-nanmean(cdata_cp(ipoint,2,:))];
%         reliability.test1.camera.Z.values = [reliability.test1.camera.Z.values cdata_cp(ipoint,3,itrial)-nanmean(cdata_cp(ipoint,3,:))];
%     end
% end
% reliability.test1.camera.X.std = nanstd(reliability.test1.camera.X.values);
% reliability.test1.camera.Y.std = nanstd(reliability.test1.camera.Y.values);
% reliability.test1.camera.Z.std = nanstd(reliability.test1.camera.Z.values);
% 
% % Assess reliability as standard deviation around the mean value
% % /orientation
% reliability.test1.camera.A1.values = [];
% reliability.test1.camera.A2.values = [];
% reliability.test1.camera.A3.values = [];
% for ipoint = 1:19
%     for itrial = 1:5
%         reliability.test1.camera.A1.values = [reliability.test1.camera.A1.values cdata_cr(ipoint,1,itrial)-nanmean(cdata_cr(ipoint,1,:))];
%         reliability.test1.camera.A2.values = [reliability.test1.camera.A2.values cdata_cr(ipoint,2,itrial)-nanmean(cdata_cr(ipoint,2,:))];
%         reliability.test1.camera.A3.values = [reliability.test1.camera.A3.values cdata_cr(ipoint,3,itrial)-nanmean(cdata_cr(ipoint,3,:))];
%     end
% end
% reliability.test1.camera.A1.std = nanstd(reliability.test1.camera.A1.values);
% reliability.test1.camera.A2.std = nanstd(reliability.test1.camera.A2.values);
% reliability.test1.camera.A3.std = nanstd(reliability.test1.camera.A3.values);
% 
% %% ------------------------------------------------------------------------
% % RELIABILITY / TEST 2 / 10x 90° flexion of axis 4
% % -------------------------------------------------------------------------
% clearvars -except Folder reliability;
% 
% % Flange position and orientation /robot
% rdata_rp = [];
% rdata_rr = [];
% robFiles = dir('*sessionname-test_unitary_axis4_+90_90-*');
% for ifile = 2:size(robFiles,1) % First file is not related to the repeatibility test
%     robFile  = robFiles(ifile).name;
%     temp     = load(robFile);
%     rdata_rp = [rdata_rp; temp.records.values(end-1,9:11)*1e3]; % Column related to the cartesian position
%     rdata_rr = [rdata_rr; rad2deg(temp.records.values(end-1,12:14))]; % Column related to the cartesian orientation
%     clear robFile temp;
% end
% rdata_rp = permute(rdata_rp,[3,2,1]);
% rdata_rr = permute(rdata_rr,[3,2,1]);
% clear ifile robFiles;
% 
% % Flange position and orientation /camera
% camFile  = 'Test_Axis1_+90_Axis4_+90_10repetitions.mat';
% temp     = load(camFile);
% fileName = fieldnames(temp);
% data     = temp.(fileName{1});
% clear camFile temp fileName;
% eventInd = [];
% for ievent = 2:2:size(data.Events,1)
%     eventInd = [eventInd data.Events(ievent).Frame];
% end
% eventInd = fix(eventInd);
% for ievent = 1:length(eventInd)
%     if ~isnan(eventInd(ievent))
%         cdata_cp(1,:,ievent) = permute(data.RigidBodies.Positions(2,:,eventInd(ievent)),[3,2,1]);
%         temp_cr              = permute(data.RigidBodies.Rotations(2,:,eventInd(ievent)),[3,2,1]);
%         R_cr                 = [[temp_cr(1) temp_cr(2) temp_cr(3)]',[temp_cr(4) temp_cr(5) temp_cr(6)]',[temp_cr(7) temp_cr(8) temp_cr(9)]'];
%         Euler                = rad2deg(R2mobileXYZ_array3(R_cr));
%         cdata_cr(1,:,ievent) = Euler;
%     else
%         cdata_cp(1,:,ievent) = NaN(1,3);
%         cdata_cr(1,:,ievent) = NaN(1,3);
%     end
% end
% clear data ievent temp_cr;
% 
% % Assess reliability as standard deviation around the mean value
% % /position
% reliability.test2.robot.X.values  = [];
% reliability.test2.robot.Y.values  = [];
% reliability.test2.robot.Z.values  = [];
% reliability.test2.camera.X.values = [];
% reliability.test2.camera.Y.values = [];
% reliability.test2.camera.Z.values = [];
% for ipoint = 1
%     for itrial = 1:10
%         reliability.test2.robot.X.values  = [reliability.test2.robot.X.values rdata_rp(ipoint,1,itrial)-mean(rdata_rp(ipoint,1,:))];
%         reliability.test2.robot.Y.values  = [reliability.test2.robot.Y.values rdata_rp(ipoint,2,itrial)-mean(rdata_rp(ipoint,2,:))];
%         reliability.test2.robot.Z.values  = [reliability.test2.robot.Z.values rdata_rp(ipoint,3,itrial)-mean(rdata_rp(ipoint,3,:))];
%         reliability.test2.camera.X.values = [reliability.test2.camera.X.values cdata_cp(ipoint,1,itrial)-nanmean(cdata_cp(ipoint,1,:))];
%         reliability.test2.camera.Y.values = [reliability.test2.camera.Y.values cdata_cp(ipoint,2,itrial)-nanmean(cdata_cp(ipoint,2,:))];
%         reliability.test2.camera.Z.values = [reliability.test2.camera.Z.values cdata_cp(ipoint,3,itrial)-nanmean(cdata_cp(ipoint,3,:))];
%     end
% end
% reliability.test2.robot.X.std  = std(reliability.test2.robot.X.values);
% reliability.test2.robot.Y.std  = std(reliability.test2.robot.Y.values);
% reliability.test2.robot.Z.std  = std(reliability.test2.robot.Z.values);
% reliability.test2.camera.X.std = nanstd(reliability.test2.camera.X.values);
% reliability.test2.camera.Y.std = nanstd(reliability.test2.camera.Y.values);
% reliability.test2.camera.Z.std = nanstd(reliability.test2.camera.Z.values);
% 
% % Assess reliability as standard deviation around the mean value
% % /orientation
% reliability.test2.robot.A1.values  = [];
% reliability.test2.robot.A2.values  = [];
% reliability.test2.robot.A3.values  = [];
% reliability.test2.camera.A1.values = [];
% reliability.test2.camera.A2.values = [];
% reliability.test2.camera.A3.values = [];
% for ipoint = 1
%     for itrial = 1:10
%         reliability.test2.robot.A1.values  = [reliability.test2.robot.X.values rdata_rr(ipoint,1,itrial)-mean(rdata_rr(ipoint,1,:))];
%         reliability.test2.robot.A2.values  = [reliability.test2.robot.Y.values rdata_rr(ipoint,2,itrial)-mean(rdata_rr(ipoint,2,:))];
%         reliability.test2.robot.A3.values  = [reliability.test2.robot.Z.values rdata_rr(ipoint,3,itrial)-mean(rdata_rr(ipoint,3,:))];
%         reliability.test2.camera.A1.values = [reliability.test2.camera.X.values cdata_cr(ipoint,1,itrial)-nanmean(cdata_cr(ipoint,1,:))];
%         reliability.test2.camera.A2.values = [reliability.test2.camera.Y.values cdata_cr(ipoint,2,itrial)-nanmean(cdata_cr(ipoint,2,:))];
%         reliability.test2.camera.A3.values = [reliability.test2.camera.Z.values cdata_cr(ipoint,3,itrial)-nanmean(cdata_cr(ipoint,3,:))];
%     end
% end
% reliability.test2.robot.A1.std  = std(reliability.test2.robot.A1.values);
% reliability.test2.robot.A2.std  = std(reliability.test2.robot.A2.values);
% reliability.test2.robot.A3.std  = std(reliability.test2.robot.A3.values);
% reliability.test2.camera.A1.std = nanstd(reliability.test2.camera.A1.values);
% reliability.test2.camera.A2.std = nanstd(reliability.test2.camera.A2.values);
% reliability.test2.camera.A3.std = nanstd(reliability.test2.camera.A3.values);
% 
% %% ------------------------------------------------------------------------
% % RELIABILITY / TEST 3 / 5x 19 points calibration
% % -------------------------------------------------------------------------
% clearvars -except Folder reliability;
% 
% % Flange position and orientation /robot
% rdata_rp = nan(19,3,5); % Flange position in robot base coordinate system
% rdata_rr = nan(19,3,5); % Flange orientation in robot base coordinate system
% robFiles = dir('calibration-*19points_trial*.mat');
% for ifile = 1:size(robFiles,1)
%     robFile = robFiles(ifile).name;
%     temp    = load(robFile);
%     rdata_rp(:,:,ifile) = temp.calibration_data.robot_eef_pose(:,1:3)*1e3; % m to mm
%     rdata_rr(:,:,ifile) = rad2deg(temp.calibration_data.robot_eef_pose(:,4:6)); % rad to deg
%     clear robFile temp;
% end
% clear ifile robFiles;
% 
% % Flange position and orientation /camera
% cdata_cp = nan(19,3,5);
% cdata_cr = nan(19,3,5);
% camFiles = dir('Calib_19points_trial*.mat');
% for ifile = 1:size(camFiles,1)
%     camFile  = camFiles(ifile).name;
%     temp     = load(camFile);
%     fileName = fieldnames(temp);
%     data     = temp.(fileName{1});
%     eventInd = [];
%     for ievent = 1:size(data.Events,1)
%         if ievent == 2
%             eventInd = [eventInd NaN]; % Position 2 not seen
%             eventInd = [eventInd data.Events(ievent).Frame];
%         elseif ievent == 4
%             eventInd = [eventInd NaN]; % Position 5 not seen
%             eventInd = [eventInd data.Events(ievent).Frame];
%         else
%             eventInd = [eventInd data.Events(ievent).Frame];
%         end
%     end
%     eventInd = fix(eventInd);
%     for ievent = 1:length(eventInd)
%         if ~isnan(eventInd(ievent))
%             cdata_cp(ievent,:,ifile) = permute(data.RigidBodies.Positions(2,:,eventInd(ievent)),[3,2,1]);
%             temp_cr                  = permute(data.RigidBodies.Rotations(2,:,eventInd(ievent)),[3,2,1]);
%             R_cr                     = [[temp_cr(1) temp_cr(2) temp_cr(3)]',[temp_cr(4) temp_cr(5) temp_cr(6)]',[temp_cr(7) temp_cr(8) temp_cr(9)]'];
%             Euler                    = rad2deg(R2mobileXYZ_array3(R_cr));
%             cdata_cr(ievent,:,ifile) = Euler;
%         else
%             cdata_cp(ievent,:,ifile) = NaN(1,3);
%             cdata_cr(ievent,:,ifile) = NaN(1,3);
%         end
%     end
%     clear camFile temp eventInd data fileName ievent temp_cr;
% end
% 
% % Assess reliability as standard deviation around the mean value
% % /position
% reliability.test3.robot.X.values  = [];
% reliability.test3.robot.Y.values  = [];
% reliability.test3.robot.Z.values  = [];
% reliability.test3.camera.X.values = [];
% reliability.test3.camera.Y.values = [];
% reliability.test3.camera.Z.values = [];
% for ipoint = 1:19
%     for itrial = 1:5
%         reliability.test3.robot.X.values  = [reliability.test3.robot.X.values rdata_rp(ipoint,1,itrial)-mean(rdata_rp(ipoint,1,:))];
%         reliability.test3.robot.Y.values  = [reliability.test3.robot.Y.values rdata_rp(ipoint,2,itrial)-mean(rdata_rp(ipoint,2,:))];
%         reliability.test3.robot.Z.values  = [reliability.test3.robot.Z.values rdata_rp(ipoint,3,itrial)-mean(rdata_rp(ipoint,3,:))];
%         reliability.test3.camera.X.values = [reliability.test3.camera.X.values cdata_cp(ipoint,1,itrial)-nanmean(cdata_cp(ipoint,1,:))];
%         reliability.test3.camera.Y.values = [reliability.test3.camera.Y.values cdata_cp(ipoint,2,itrial)-nanmean(cdata_cp(ipoint,2,:))];
%         reliability.test3.camera.Z.values = [reliability.test3.camera.Z.values cdata_cp(ipoint,3,itrial)-nanmean(cdata_cp(ipoint,3,:))];
%     end
% end
% reliability.test3.robot.X.std  = std(reliability.test3.robot.X.values);
% reliability.test3.robot.Y.std  = std(reliability.test3.robot.Y.values);
% reliability.test3.robot.Z.std  = std(reliability.test3.robot.Z.values);
% reliability.test3.camera.X.std = nanstd(reliability.test3.camera.X.values);
% reliability.test3.camera.Y.std = nanstd(reliability.test3.camera.Y.values);
% reliability.test3.camera.Z.std = nanstd(reliability.test3.camera.Z.values);
% 
% % Assess reliability as standard deviation around the mean value
% % /orientation
% reliability.test3.robot.A1.values  = [];
% reliability.test3.robot.A2.values  = [];
% reliability.test3.robot.A3.values  = [];
% reliability.test3.camera.A1.values = [];
% reliability.test3.camera.A2.values = [];
% reliability.test3.camera.A3.values = [];
% for ipoint = 1:19
%     for itrial = 1:5
%         reliability.test3.robot.A1.values  = [reliability.test3.robot.X.values rdata_rr(ipoint,1,itrial)-mean(rdata_rr(ipoint,1,:))];
%         reliability.test3.robot.A2.values  = [reliability.test3.robot.Y.values rdata_rr(ipoint,2,itrial)-mean(rdata_rr(ipoint,2,:))];
%         reliability.test3.robot.A3.values  = [reliability.test3.robot.Z.values rdata_rr(ipoint,3,itrial)-mean(rdata_rr(ipoint,3,:))];
%         reliability.test3.camera.A1.values = [reliability.test3.camera.X.values cdata_cr(ipoint,1,itrial)-nanmean(cdata_cr(ipoint,1,:))];
%         reliability.test3.camera.A2.values = [reliability.test3.camera.Y.values cdata_cr(ipoint,2,itrial)-nanmean(cdata_cr(ipoint,2,:))];
%         reliability.test3.camera.A3.values = [reliability.test3.camera.Z.values cdata_cr(ipoint,3,itrial)-nanmean(cdata_cr(ipoint,3,:))];
%     end
% end
% reliability.test3.robot.A1.std  = std(reliability.test3.robot.A1.values);
% reliability.test3.robot.A2.std  = std(reliability.test3.robot.A2.values);
% reliability.test3.robot.A3.std  = std(reliability.test3.robot.A3.values);
% reliability.test3.camera.A1.std = nanstd(reliability.test3.camera.A1.values);
% reliability.test3.camera.A2.std = nanstd(reliability.test3.camera.A2.values);
% reliability.test3.camera.A3.std = nanstd(reliability.test3.camera.A3.values);
% 
% %% ------------------------------------------------------------------------
% % RELIABILITY / TEST 4 / 1x 19 points calibration, QTM vs David storage
% % -------------------------------------------------------------------------
% clearvars -except Folder reliability;
% 
% % Flange position and orientation /robot
% rdata_rp = nan(19,3,1); % Flange position in robot base coordinate system
% rdata_cp = nan(19,3,1); % Flange position in camera coordinate system
% rdata_rr = nan(19,3,1); % Flange orientation in robot base coordinate system
% robFiles = dir('calibration-*19points_trialbonus.mat');
% for ifile = 1:size(robFiles,1)
%     robFile = robFiles(ifile).name;
%     temp    = load(robFile);
%     rdata_rp(:,:,ifile) = temp.calibration_data.robot_eef_pose(:,1:3)*1e3; % m to mm
%     rdata_cp(:,:,ifile) = temp.calibration_data.camera_eef_pose(:,1:3)*1e3; % m to mm
%     rdata_rr(:,:,ifile) = rad2deg(temp.calibration_data.robot_eef_pose(:,4:6)); % rad to deg
%     clear robFile temp;
% end
% clear ifile robFiles;
% 
% % Flange position and orientation /camera
% cdata_cp = nan(19,3,1);
% cdata_cr = nan(19,3,1);
% camFiles = dir('Calib_19points_trial03.mat');
% for ifile = 1:size(camFiles,1)
%     camFile  = camFiles(ifile).name;
%     temp     = load(camFile);
%     fileName = fieldnames(temp);
%     data     = temp.(fileName{1});
%     eventInd = [];
%     for ievent = 1:size(data.Events,1)
%         if ievent == 2
%             eventInd = [eventInd NaN]; % Position 2 not seen
%             eventInd = [eventInd data.Events(ievent).Frame];
%         elseif ievent == 4
%             eventInd = [eventInd NaN]; % Position 5 not seen
%             eventInd = [eventInd data.Events(ievent).Frame];
%         else
%             eventInd = [eventInd data.Events(ievent).Frame];
%         end
%     end
%     eventInd = fix(eventInd);
%     for ievent = 1:length(eventInd)
%         if ~isnan(eventInd(ievent))
%             cdata_cp(ievent,:,ifile) = permute(data.RigidBodies.Positions(2,:,eventInd(ievent)),[3,2,1]);
%             temp_cr                  = permute(data.RigidBodies.Rotations(2,:,eventInd(ievent)),[3,2,1]);
%             R_cr                     = [[temp_cr(1) temp_cr(2) temp_cr(3)]',[temp_cr(4) temp_cr(5) temp_cr(6)]',[temp_cr(7) temp_cr(8) temp_cr(9)]'];
%             Euler                    = rad2deg(R2mobileXYZ_array3(R_cr));
%             cdata_cr(ievent,:,ifile) = Euler;
%         else
%             cdata_cp(ievent,:,ifile) = NaN(1,3);
%             cdata_cr(ievent,:,ifile) = NaN(1,3);
%         end
%     end
%     clear camFile temp eventInd data fileName ievent temp_cr;
% end
% 
% % Assess difference between TPC-IP reported camera values and file-stored
% % camera values
% % /position
% reliability.test4.rcamera.X.values  = [];
% reliability.test4.rcamera.Y.values  = [];
% reliability.test4.rcamera.Z.values  = [];
% reliability.test4.camera.X.values = [];
% reliability.test4.camera.Y.values = [];
% reliability.test4.camera.Z.values = [];
% for ipoint = 1:19
%     for itrial = 1
%         reliability.test4.rcamera.X.values  = [reliability.test4.rcamera.X.values rdata_cp(ipoint,1,itrial)];
%         reliability.test4.rcamera.Y.values  = [reliability.test4.rcamera.Y.values rdata_cp(ipoint,2,itrial)];
%         reliability.test4.rcamera.Z.values  = [reliability.test4.rcamera.Z.values rdata_cp(ipoint,3,itrial)];
%         reliability.test4.camera.X.values = [reliability.test4.camera.X.values cdata_cp(ipoint,1,itrial)];
%         reliability.test4.camera.Y.values = [reliability.test4.camera.Y.values cdata_cp(ipoint,2,itrial)];
%         reliability.test4.camera.Z.values = [reliability.test4.camera.Z.values cdata_cp(ipoint,3,itrial)];
%     end
% end
% reliability.test4.difference.X.values = reliability.test4.rcamera.X.values-reliability.test4.camera.X.values;
% reliability.test4.difference.Y.values = reliability.test4.rcamera.Y.values-reliability.test4.camera.Y.values;
% reliability.test4.difference.Z.values = reliability.test4.rcamera.Z.values-reliability.test4.camera.Z.values;
% reliability.test4.difference.X.max    = max(abs(reliability.test4.difference.X.values));
% reliability.test4.difference.Y.max    = max(abs(reliability.test4.difference.Y.values));
% reliability.test4.difference.Z.max    = max(abs(reliability.test4.difference.Z.values));
% 
% %% ------------------------------------------------------------------------
% % VALIDITY / TEST 1 / Find rigid transformation between robot and camera
% % coordinate systems
% % -------------------------------------------------------------------------
% clearvars -except Folder reliability;
% 
% % Initialisation
% rdata_rp = [];
% rdata_rr = [];
% cdata_cp = [];
% cdata_cr = [];
% 
% % Flange position and orientation /robot
% % -------------------------------------------------------------------------
% % % Axis 2 -90°
% % robFiles = dir('*sessionname-test_unitary_axis2_-*.mat');
% % for ifile = [1 3 4 5 6 7 8 9]
% %     robFile  = robFiles(ifile).name;
% %     temp     = load(robFile);
% %     rdata_rp = [rdata_rp; temp.records.values(end-1,9:11)*1e3]; % Column related to the cartesian position
% %     rdata_rr = [rdata_rr; rad2deg(temp.records.values(end-1,12:14))]; % Column related to the cartesian orientation
% %     clear robFile temp;
% % end
% % clear ifile robFiles;
% % Axis 4 +90°
% robFiles = dir('*sessionname-test_unitary_axis4_+*.mat');
% for ifile = [1 3 4 5 6 7 8 9]
%     robFile  = robFiles(ifile).name;
%     temp     = load(robFile);
%     rdata_rp = [rdata_rp; temp.records.values(end-1,9:11)*1e3]; % Column related to the cartesian position
%     rdata_rr = [rdata_rr; rad2deg(temp.records.values(end-1,12:14))]; % Column related to the cartesian orientation
%     clear robFile temp;
% end
% clear ifile robFiles;
% % Axis 4 -90°
% robFiles = dir('*sessionname-test_unitary_axis4_-*.mat');
% for ifile = 1:9
%     robFile  = robFiles(ifile).name;
%     temp     = load(robFile);
%     rdata_rp = [rdata_rp; temp.records.values(end-1,9:11)*1e3]; % Column related to the cartesian position
%     rdata_rr = [rdata_rr; rad2deg(temp.records.values(end-1,12:14))]; % Column related to the cartesian orientation
%     clear robFile temp;
% end
% clear ifile robFiles;
% % Axis 6 +90°
% robFiles = dir('*sessionname-test_unitary_axis6_+*.mat');
% for ifile = 1:9
%     robFile  = robFiles(ifile).name;
%     temp     = load(robFile);
%     rdata_rp = [rdata_rp; temp.records.values(end-1,9:11)*1e3]; % Column related to the cartesian position
%     rdata_rr = [rdata_rr; rad2deg(temp.records.values(end-1,12:14))]; % Column related to the cartesian orientation
%     clear robFile temp;
% end
% clear ifile robFiles;
% % Axis 6 -90°
% robFiles = dir('*sessionname-test_unitary_axis6_-*.mat');
% for ifile = 1:9
%     robFile  = robFiles(ifile).name;
%     temp     = load(robFile);
%     rdata_rp = [rdata_rp; temp.records.values(end-1,9:11)*1e3]; % Column related to the cartesian position
%     rdata_rr = [rdata_rr; rad2deg(temp.records.values(end-1,12:14))]; % Column related to the cartesian orientation
%     clear robFile temp;
% end
% clear ifile robFiles;
% % % Axis 1 +90° / Axis 2 +90°
% % robFiles = dir('*sessionname-test_unitary_axis2_+*_90*.mat');
% % for ifile = [1 2 8]
% %     robFile  = robFiles(ifile).name;
% %     temp     = load(robFile);
% %     rdata_rp = [rdata_rp; temp.records.values(end-1,9:11)*1e3]; % Column related to the cartesian position
% %     rdata_rr = [rdata_rr; rad2deg(temp.records.values(end-1,12:14))]; % Column related to the cartesian orientation
% %     clear robFile temp;
% % end
% % clear ifile robFiles;
% % % Axis 1 +90° / Axis 2 -90°
% % robFiles = dir('*sessionname-test_unitary_axis2_-*_90*.mat');
% % for ifile = 1:9
% %     robFile  = robFiles(ifile).name;
% %     temp     = load(robFile);
% %     rdata_rp = [rdata_rp; temp.records.values(end-1,9:11)*1e3]; % Column related to the cartesian position
% %     rdata_rr = [rdata_rr; rad2deg(temp.records.values(end-1,12:14))]; % Column related to the cartesian orientation
% %     clear robFile temp;
% % end
% % clear ifile robFiles;
% % Axis 1 +90° / Axis 4 +90°
% robFiles = dir('*sessionname-test_unitary_axis4_+*_90*.mat');
% for ifile = 1:9
%     robFile  = robFiles(ifile).name;
%     temp     = load(robFile);
%     rdata_rp = [rdata_rp; temp.records.values(end-1,9:11)*1e3]; % Column related to the cartesian position
%     rdata_rr = [rdata_rr; rad2deg(temp.records.values(end-1,12:14))]; % Column related to the cartesian orientation
%     clear robFile temp;
% end
% clear ifile robFiles;
% % Axis 1 +90° / Axis 4 -90°
% robFiles = dir('*sessionname-test_unitary_axis4_-*_90*.mat');
% for ifile = 1:9
%     robFile  = robFiles(ifile).name;
%     temp     = load(robFile);
%     rdata_rp = [rdata_rp; temp.records.values(end-1,9:11)*1e3]; % Column related to the cartesian position
%     rdata_rr = [rdata_rr; rad2deg(temp.records.values(end-1,12:14))]; % Column related to the cartesian orientation
%     clear robFile temp;
% end
% clear ifile robFiles;
% % Axis 1 +90° / Axis 6 +90°
% robFiles = dir('*sessionname-test_unitary_axis6_+*_90*.mat');
% for ifile = 1:9
%     robFile  = robFiles(ifile).name;
%     temp     = load(robFile);
%     rdata_rp = [rdata_rp; temp.records.values(end-1,9:11)*1e3]; % Column related to the cartesian position
%     rdata_rr = [rdata_rr; rad2deg(temp.records.values(end-1,12:14))]; % Column related to the cartesian orientation
%     clear robFile temp;
% end
% clear ifile robFiles;
% % Axis 1 +90° / Axis 6 -90°
% robFiles = dir('*sessionname-test_unitary_axis6_-*_90*.mat');
% for ifile = 1:9
%     robFile  = robFiles(ifile).name;
%     temp     = load(robFile);
%     rdata_rp = [rdata_rp; temp.records.values(end-1,9:11)*1e3]; % Column related to the cartesian position
%     rdata_rr = [rdata_rr; rad2deg(temp.records.values(end-1,12:14))]; % Column related to the cartesian orientation
%     clear robFile temp;
% end
% clear ifile robFiles;
% 
% % Flange position and orientation /cameras
% % -------------------------------------------------------------------------
% % % Axis 2 -90°
% % camFile = 'Test_Axis2_-90.mat';
% % temp     = load(camFile);
% % fileName = fieldnames(temp);
% % data     = temp.(fileName{1});
% % eventInd = [];
% % for ievent = 2:length(data.Events)
% %     eventInd = [eventInd data.Events(ievent).Frame];
% % end
% % eventInd = fix(eventInd);
% % for ievent = 1:length(eventInd)
% %     cdata_cp = [cdata_cp; permute(data.RigidBodies.Positions(2,:,eventInd(ievent)),[3,2,1])];
% %     temp_cr  = permute(data.RigidBodies.Rotations(2,:,eventInd(ievent)),[3,2,1]);
% %     R_cr     = [[temp_cr(1) temp_cr(2) temp_cr(3)]',[temp_cr(4) temp_cr(5) temp_cr(6)]',[temp_cr(7) temp_cr(8) temp_cr(9)]'];
% %     Euler    = rad2deg(R2mobileXYZ_array3(R_cr));
% %     cdata_cr = [cdata_cr; Euler];
% % end
% % clear camFile temp eventInd data fileName ievent temp_cr;
% % Axis 4 +90°
% camFile = 'Test_Axis4_+90.mat';
% temp     = load(camFile);
% fileName = fieldnames(temp);
% data     = temp.(fileName{1});
% eventInd = [];
% for ievent = 2:length(data.Events)
%     eventInd = [eventInd data.Events(ievent).Frame];
% end
% eventInd = fix(eventInd);
% for ievent = 1:length(eventInd)
%     cdata_cp = [cdata_cp; permute(data.RigidBodies.Positions(2,:,eventInd(ievent)),[3,2,1])];
%     temp_cr  = permute(data.RigidBodies.Rotations(2,:,eventInd(ievent)),[3,2,1]);
%     R_cr     = [[temp_cr(1) temp_cr(2) temp_cr(3)]',[temp_cr(4) temp_cr(5) temp_cr(6)]',[temp_cr(7) temp_cr(8) temp_cr(9)]'];
%     Euler    = rad2deg(R2mobileXYZ_array3(R_cr));
%     cdata_cr = [cdata_cr; Euler];
% end
% clear camFile temp eventInd data fileName ievent temp_cr;
% % Axis 4 -90°
% camFile = 'Test_Axis4_-90.mat';
% temp     = load(camFile);
% fileName = fieldnames(temp);
% data     = temp.(fileName{1});
% eventInd = [];
% for ievent = 2:length(data.Events)
%     eventInd = [eventInd data.Events(ievent).Frame];
% end
% eventInd = fix(eventInd);
% for ievent = 1:length(eventInd)
%     cdata_cp = [cdata_cp; permute(data.RigidBodies.Positions(2,:,eventInd(ievent)),[3,2,1])];
%     temp_cr  = permute(data.RigidBodies.Rotations(2,:,eventInd(ievent)),[3,2,1]);
%     R_cr     = [[temp_cr(1) temp_cr(2) temp_cr(3)]',[temp_cr(4) temp_cr(5) temp_cr(6)]',[temp_cr(7) temp_cr(8) temp_cr(9)]'];
%     Euler    = rad2deg(R2mobileXYZ_array3(R_cr));
%     cdata_cr = [cdata_cr; Euler];
% end
% clear camFile temp eventInd data fileName ievent temp_cr;
% % Axis 6 +90°
% camFile = 'Test_Axis6_+90.mat';
% temp     = load(camFile);
% fileName = fieldnames(temp);
% data     = temp.(fileName{1});
% eventInd = [];
% for ievent = 2:length(data.Events)
%     eventInd = [eventInd data.Events(ievent).Frame];
% end
% eventInd = fix(eventInd);
% for ievent = 1:length(eventInd)
%     cdata_cp = [cdata_cp; permute(data.RigidBodies.Positions(2,:,eventInd(ievent)),[3,2,1])];
%     temp_cr  = permute(data.RigidBodies.Rotations(2,:,eventInd(ievent)),[3,2,1]);
%     R_cr     = [[temp_cr(1) temp_cr(2) temp_cr(3)]',[temp_cr(4) temp_cr(5) temp_cr(6)]',[temp_cr(7) temp_cr(8) temp_cr(9)]'];
%     Euler    = rad2deg(R2mobileXYZ_array3(R_cr));
%     cdata_cr = [cdata_cr; Euler];
% end
% clear camFile temp eventInd data fileName ievent temp_cr;
% % Axis 6 -90°
% camFile = 'Test_Axis6_-90.mat';
% temp     = load(camFile);
% fileName = fieldnames(temp);
% data     = temp.(fileName{1});
% eventInd = [];
% for ievent = 2:length(data.Events)
%     eventInd = [eventInd data.Events(ievent).Frame];
% end
% eventInd = fix(eventInd);
% for ievent = 1:length(eventInd)
%     cdata_cp = [cdata_cp; permute(data.RigidBodies.Positions(2,:,eventInd(ievent)),[3,2,1])];
%     temp_cr  = permute(data.RigidBodies.Rotations(2,:,eventInd(ievent)),[3,2,1]);
%     R_cr     = [[temp_cr(1) temp_cr(2) temp_cr(3)]',[temp_cr(4) temp_cr(5) temp_cr(6)]',[temp_cr(7) temp_cr(8) temp_cr(9)]'];
%     Euler    = rad2deg(R2mobileXYZ_array3(R_cr));
%     cdata_cr = [cdata_cr; Euler];
% end
% clear camFile temp eventInd data fileName ievent temp_cr;
% % % Axis 1 +90° / Axis 2 +90°
% % camFile = 'Test_Axis1_+90_Axis2_+90.mat';
% % temp     = load(camFile);
% % fileName = fieldnames(temp);
% % data     = temp.(fileName{1});
% % eventInd = [];
% % for ievent = 2:length(data.Events)
% %     eventInd = [eventInd data.Events(ievent).Frame];
% % end
% % eventInd = fix(eventInd);
% % for ievent = 1:length(eventInd)
% %     cdata_cp = [cdata_cp; permute(data.RigidBodies.Positions(2,:,eventInd(ievent)),[3,2,1])];
% %     temp_cr  = permute(data.RigidBodies.Rotations(2,:,eventInd(ievent)),[3,2,1]);
% %     R_cr     = [[temp_cr(1) temp_cr(2) temp_cr(3)]',[temp_cr(4) temp_cr(5) temp_cr(6)]',[temp_cr(7) temp_cr(8) temp_cr(9)]'];
% %     Euler    = rad2deg(R2mobileXYZ_array3(R_cr));
% %     cdata_cr = [cdata_cr; Euler];
% % end
% % clear camFile temp eventInd data fileName ievent temp_cr;
% % % Axis 1 +90° / Axis 2 -90°
% % camFile = 'Test_Axis1_+90_Axis2_-90.mat';
% % temp     = load(camFile);
% % fileName = fieldnames(temp);
% % data     = temp.(fileName{1});
% % eventInd = [];
% % for ievent = 2:length(data.Events)
% %     eventInd = [eventInd data.Events(ievent).Frame];
% % end
% % eventInd = fix(eventInd);
% % for ievent = 1:length(eventInd)
% %     cdata_cp = [cdata_cp; permute(data.RigidBodies.Positions(2,:,eventInd(ievent)),[3,2,1])];
% %     temp_cr  = permute(data.RigidBodies.Rotations(2,:,eventInd(ievent)),[3,2,1]);
% %     R_cr     = [[temp_cr(1) temp_cr(2) temp_cr(3)]',[temp_cr(4) temp_cr(5) temp_cr(6)]',[temp_cr(7) temp_cr(8) temp_cr(9)]'];
% %     Euler    = rad2deg(R2mobileXYZ_array3(R_cr));
% %     cdata_cr = [cdata_cr; Euler];
% % end
% % clear camFile temp eventInd data fileName ievent temp_cr;
% % Axis 1 +90° / Axis 4 +90°
% camFile = 'Test_Axis1_+90_Axis4_+90.mat';
% temp     = load(camFile);
% fileName = fieldnames(temp);
% data     = temp.(fileName{1});
% eventInd = [];
% for ievent = 2:length(data.Events)
%     eventInd = [eventInd data.Events(ievent).Frame];
% end
% eventInd = fix(eventInd);
% for ievent = 1:length(eventInd)
%     cdata_cp = [cdata_cp; permute(data.RigidBodies.Positions(2,:,eventInd(ievent)),[3,2,1])];
%     temp_cr  = permute(data.RigidBodies.Rotations(2,:,eventInd(ievent)),[3,2,1]);
%     R_cr     = [[temp_cr(1) temp_cr(2) temp_cr(3)]',[temp_cr(4) temp_cr(5) temp_cr(6)]',[temp_cr(7) temp_cr(8) temp_cr(9)]'];
%     Euler    = rad2deg(R2mobileXYZ_array3(R_cr));
%     cdata_cr = [cdata_cr; Euler];
% end
% clear camFile temp eventInd data fileName ievent temp_cr;
% % Axis 1 +90° / Axis 4 -90°
% camFile = 'Test_Axis1_+90_Axis4_-90.mat';
% temp     = load(camFile);
% fileName = fieldnames(temp);
% data     = temp.(fileName{1});
% eventInd = [];
% for ievent = 2:length(data.Events)
%     eventInd = [eventInd data.Events(ievent).Frame];
% end
% eventInd = fix(eventInd);
% for ievent = 1:length(eventInd)
%     cdata_cp = [cdata_cp; permute(data.RigidBodies.Positions(2,:,eventInd(ievent)),[3,2,1])];
%     temp_cr  = permute(data.RigidBodies.Rotations(2,:,eventInd(ievent)),[3,2,1]);
%     R_cr     = [[temp_cr(1) temp_cr(2) temp_cr(3)]',[temp_cr(4) temp_cr(5) temp_cr(6)]',[temp_cr(7) temp_cr(8) temp_cr(9)]'];
%     Euler    = rad2deg(R2mobileXYZ_array3(R_cr));
%     cdata_cr = [cdata_cr; Euler];
% end
% clear camFile temp eventInd data fileName ievent temp_cr;
% % Axis 1 +90° / Axis 6 +90°
% camFile = 'Test_Axis1_+90_Axis6_+90.mat';
% temp     = load(camFile);
% fileName = fieldnames(temp);
% data     = temp.(fileName{1});
% eventInd = [];
% for ievent = 2:length(data.Events)
%     eventInd = [eventInd data.Events(ievent).Frame];
% end
% eventInd = fix(eventInd);
% for ievent = 1:length(eventInd)
%     cdata_cp = [cdata_cp; permute(data.RigidBodies.Positions(2,:,eventInd(ievent)),[3,2,1])];
%     temp_cr  = permute(data.RigidBodies.Rotations(2,:,eventInd(ievent)),[3,2,1]);
%     R_cr     = [[temp_cr(1) temp_cr(2) temp_cr(3)]',[temp_cr(4) temp_cr(5) temp_cr(6)]',[temp_cr(7) temp_cr(8) temp_cr(9)]'];
%     Euler    = rad2deg(R2mobileXYZ_array3(R_cr));
%     cdata_cr = [cdata_cr; Euler];
% end
% clear camFile temp eventInd data fileName ievent temp_cr;
% % Axis 1 +90° / Axis 6 -90°
% camFile = 'Test_Axis1_+90_Axis6_-90.mat';
% temp     = load(camFile);
% fileName = fieldnames(temp);
% data     = temp.(fileName{1});
% eventInd = [];
% for ievent = 2:length(data.Events)
%     eventInd = [eventInd data.Events(ievent).Frame];
% end
% eventInd = fix(eventInd);
% for ievent = 1:length(eventInd)
%     cdata_cp = [cdata_cp; permute(data.RigidBodies.Positions(2,:,eventInd(ievent)),[3,2,1])];
%     temp_cr  = permute(data.RigidBodies.Rotations(2,:,eventInd(ievent)),[3,2,1]);
%     R_cr     = [[temp_cr(1) temp_cr(2) temp_cr(3)]',[temp_cr(4) temp_cr(5) temp_cr(6)]',[temp_cr(7) temp_cr(8) temp_cr(9)]'];
%     Euler    = rad2deg(R2mobileXYZ_array3(R_cr));
%     cdata_cr = [cdata_cr; Euler];
% end
% clear camFile temp eventInd data fileName ievent temp_cr;
% 
% figure; hold on; axis equal;
% plot3(cdata_cp(:,1),cdata_cp(:,2),cdata_cp(:,3),'Marker','.','Markersize',15,'Color','red');
% plot3(rdata_rp(:,1),rdata_rp(:,2),rdata_rp(:,3),'Marker','.','Markersize',15,'Color','blue');
% 
% [R,d,rms] = soder(cdata_cp,rdata_rp);
% s = 1;
% 
% % cdata_cp2 = (s*R*cdata_cp'+d)';% 
% % figure; hold on; axis equal;
% % plot3(cdata_cp2(:,1),cdata_cp2(:,2),cdata_cp2(:,3),'Marker','.','Markersize',15,'Color','red');
% % plot3(rdata_rp(:,1),rdata_rp(:,2),rdata_rp(:,3),'Marker','.','Markersize',15,'Color','blue');
% 
% % load('calibration-2023-03-22-14-23-45-424.mat');5
% 
% %% ------------------------------------------------------------------------
% % VALIDITY / TEST 2 / Test on calibration data (19 points)
% % -------------------------------------------------------------------------
% clearvars -except Folder reliability R d s rms;
% 
% % Initialisation
% rdata_rp = [];
% rdata_rr = [];
% cdata_cp = [];
% cdata_cr = [];
% 
% % Flange position and orientation /robot
% % -------------------------------------------------------------------------
% robFiles = dir('calibration-2023-03-23-10-18-43-667_19points_trial01.mat');
% for ifile = 1:size(robFiles,1)
%     robFile = robFiles(ifile).name;
%     temp    = load(robFile);
%     rdata_rp(:,:,ifile) = temp.calibration_data.robot_eef_pose([1 3 4 6:19],1:3)*1e3; % m to mm
%     rdata_rr(:,:,ifile) = rad2deg(temp.calibration_data.robot_eef_pose([1 3 4 6:19],4:6)); % rad to deg
%     clear robFile temp;
% end
% clear ifile robFiles;
% 
% % Flange position and orientation /cameras
% % -------------------------------------------------------------------------
% camFile = 'Calib_19points_trial01.mat';
% temp     = load(camFile);
% fileName = fieldnames(temp);
% data     = temp.(fileName{1});
% eventInd = [];
% for ievent = 1:length(data.Events)
%     eventInd = [eventInd data.Events(ievent).Frame];
% end
% eventInd = fix(eventInd);
% for ievent = 1:length(eventInd)
%     cdata_cp = [cdata_cp; permute(data.RigidBodies.Positions(2,:,eventInd(ievent)),[3,2,1])];
%     temp_cr  = permute(data.RigidBodies.Rotations(2,:,eventInd(ievent)),[3,2,1]);
%     R_cr     = [[temp_cr(1) temp_cr(2) temp_cr(3)]',[temp_cr(4) temp_cr(5) temp_cr(6)]',[temp_cr(7) temp_cr(8) temp_cr(9)]'];
%     Euler    = rad2deg(R2mobileXYZ_array3(R_cr));
%     cdata_cr = [cdata_cr; Euler];
% end
% cdata_cp2 = (s*R*cdata_cp'+d)';
% clear camFile temp eventInd data fileName ievent temp_cr;
% 
% error1 = sqrt(sum((cdata_cp-rdata_rp).^2,1))/size(cdata_cp,1);
% error2 = sqrt(sum((cdata_cp2-rdata_rp).^2,1))/size(cdata_cp2,1);
% 
% figure; hold on; axis equal;
% plot3(cdata_cp2(:,1),cdata_cp2(:,2),cdata_cp2(:,3),'Marker','.','Markersize',15,'Color','red');
% plot3(rdata_rp(:,1),rdata_rp(:,2),rdata_rp(:,3),'Marker','.','Markersize',15,'Color','blue');
% 
% %% ------------------------------------------------------------------------
% % VALIDITY / TEST 3 / Robot achieved points vs. requested points
% % (robot assessment alone, no camera use or data)
% % -------------------------------------------------------------------------
% clearvars -except Folder reliability R d s rms;
% 
% % Initialisation
% rdata_rp = [];
% rdata_rr = [];
% cdata_cp = [];
% cdata_cr = [];
% 
% % Flange position and orientation /robot
% % -------------------------------------------------------------------------
% robFiles = dir('calibration-2023-03-30-15-39-06-743_405points.mat');
% for ifile = 1:size(robFiles,1)
%     robFile = robFiles(ifile).name;
%     temp    = load(robFile);
%     rdata_j(:,:,ifile)  = temp.calibration_data.robot_joints(:,:);
%     rdata_tp(:,:,ifile) = temp.calibration_data.required_points(:,1:3)*1e3; % m to mm
%     rdata_rp(:,:,ifile) = temp.calibration_data.robot_eef_pose(:,1:3)*1e3; % m to mm
%     rdata_rr(:,:,ifile) = rad2deg(temp.calibration_data.robot_eef_pose(:,4:6)); % rad to deg
%     clear robFile temp;
% end
% clear ifile robFiles;
% 
% initial_data.camera_to_robot  = [eye(3) zeros(3,1); 0 0 0 1];                    % Not rigid transformation
% initial_data.robot_parameters = [[0.36,0.0,0.42,0.0,0.4,0.0,0.126+0.026]*1e3,... % d, +0.026 not explained, should be 0.061 regarding the flange used on the robot
%                                  [0.0,0.0,0.0,0.0,0.0,0.0,0.0]*1e3,...           % a
%                                  0,0,0,0,0,0,0,...                               % theta
%                                  0,-pi/2,pi/2,pi/2,-pi/2,-pi/2,pi/2];            % alpha
% initial_data.robot_eef_pose   = rdata_rp;
% initial_data.required_points  = rdata_tp;
% initial_data.robot_joints     = rdata_j;
% 
% x0        = initial_data.robot_parameters;
% options   = optimoptions(@fminunc,'PlotFcns',@optimplotfval,'Algorithm','quasi-newton','HessianApproximation','bfgs','StepTolerance',1e-10,'MaxFunctionEvaluations',1e10);
% x         = fminunc(@(x)costFunction(x,initial_data,initial_data.required_points),x0,options);
% rdata_rp2 = directKinematics(x,initial_data)';
% 
% for ipoint = 1:size(cdata_cp,1)
%     error1(ipoint,:) = sqrt(sum((rdata_tp(ipoint,:)-rdata_rp(ipoint,:)).^2,1));
%     error2(ipoint,:) = sqrt(sum((rdata_tp(ipoint,:)-rdata_rp2(ipoint,:)).^2,1));
% end
% 
% figure; hold on; axis equal;
% plot3(rdata_tp(:,1),rdata_tp(:,2),rdata_tp(:,3),'Marker','.','Markersize',15,'Color','red');
% plot3(rdata_rp2(:,1),rdata_rp2(:,2),rdata_rp2(:,3),'Marker','.','Markersize',15,'Color','blue');
% 
% %% ------------------------------------------------------------------------
% % VALIDITY / TEST 4 / Robot achieved points vs. viewed camera points
% % Based on Besset et al. 2016 Advanced calibration applied to a collaborative robot
% % -------------------------------------------------------------------------
% clearvars -except Folder reliability R d s rms x_19 x_92 x_405;
% 
% % Flange position and orientation /robot
% % -------------------------------------------------------------------------
% % robFiles = dir('calibration-2023-03-31-07-34-52-765_19points.mat'); % 19 points
% % robFiles = dir('calibration-2023-03-31-08-33-14-776_92points.mat'); % 92 points
% robFiles = dir('calibration-2023-03-30-15-39-06-743_405points.mat'); % 405 points
% for ifile = 1:size(robFiles,1)
%     robFile = robFiles(ifile).name;
%     temp    = load(robFile);
%     rdata_j(:,:,ifile)  = temp.calibration_data.robot_joints(:,:);
%     rdata_rp(:,:,ifile) = temp.calibration_data.robot_eef_pose(:,1:3)*1e3; % m to mm
%     rdata_cp(:,:,ifile) = temp.calibration_data.camera_eef_pose(:,1:3)*1e3; % m to mm
%     rdata_rr(:,:,ifile) = rad2deg(temp.calibration_data.robot_eef_pose(:,4:6)); % rad to deg
%     rdata_cr(:,:,ifile) = rad2deg(temp.calibration_data.camera_eef_pose(:,4:6)); % rad to deg
%     clear robFile temp;
% end
% clear ifile robFiles;
% 
% initial_data.camera_to_robot  = [eye(3) zeros(3,1); 0 0 0 1];                    % Not rigid transformation
% initial_data.robot_parameters = [[0.36,0.0,0.42,0.0,0.4,0.0,0.126+0.026]*1e3,... % d, +0.026 not explained, should be 0.061 regarding the flange used on the robot
%                                  [0.0,0.0,0.0,0.0,0.0,0.0,0.0]*1e3,...           % a
%                                  0,0,0,0,0,0,0,...                               % theta
%                                  0,-pi/2,pi/2,pi/2,-pi/2,-pi/2,pi/2];            % alpha
% initial_data.robot_eef_pose   = [rdata_rp rdata_rr];
% rdata_cp2                     = (s*R*rdata_cp'+d)';
% initial_data.camera_eef_pose  = [rdata_cp2 rdata_cr];
% initial_data.robot_joints     = rdata_j;
% 
% x0        = initial_data.robot_parameters;
% options   = optimoptions(@fminunc,'PlotFcns',@optimplotfval,'Algorithm','quasi-newton','HessianApproximation','bfgs','StepTolerance',1e-10,'MaxFunctionEvaluations',1e10);
% x         = fminunc(@(x)costFunction(x,initial_data,initial_data.camera_eef_pose),x0,options);
% % x         = lsqnonlin(@(x)costFunction(x,initial_data,initial_data.camera_eef_pose),x0);
% rdata_rp2 = directKinematics(x,initial_data)';
% 
% % x_19  = x;
% % x_92  = x;
% x_405 = x;
% 
% for ipoint = 1:size(rdata_cp2,1)
%     error1(ipoint,:) = sqrt(sum((rdata_cp2(ipoint,:)-rdata_rp(ipoint,:)).^2,1));
%     error2(ipoint,:) = sqrt(sum((rdata_cp2(ipoint,:)-rdata_rp2(ipoint,1:3)).^2,1));
% end
% 
% figure; hold on; axis equal;
% plot3(rdata_cp2(:,1),rdata_cp2(:,2),rdata_cp2(:,3),'Marker','.','Markersize',15,'Color','red');
% plot3(rdata_rp2(:,1),rdata_rp2(:,2),rdata_rp2(:,3),'Marker','.','Markersize',15,'Color','blue');
% 
% % % Remove outliers (due to camera data gaps)
% % % bar(calibration_data.camera_eef_pose(:,3));
% % % ind = find(calibration_data.camera_eef_pose(:,3)>0.2);
% % % n   = length(ind);
% % 
% % % Set initial parameters
% % initial_data.camera_to_robot  = [eye(3) zeros(3,1); 0 0 0 1];                    % Not rigid transformation
% % initial_data.robot_parameters = [[0.36,0.0,0.42,0.0,0.4,0.0,0.126+0.026]*1e3,... % d, +0.026 not explained, should be 0.061 regarding the flange used on the robot
% %                                  [0.0,0.0,0.0,0.0,0.0,0.0,0.0]*1e3,...           % a
% %                                  0,0,0,0,0,0,0,...                               % theta
% %                                  0,-pi/2,pi/2,pi/2,-pi/2,-pi/2,pi/2];            % alpha
% % initial_data.robot_eef_pose   = calibration_data.robot_eef_pose(:,:)*1e3;%calibration_data.robot_eef_pose(ind,:)*1e3;
% % initial_data.camera_eef_pose  = calibration_data.camera_eef_pose(:,:)*1e3;%calibration_data.camera_eef_pose(ind,:)*1e3;
% % initial_data.robot_joints     = calibration_data.robot_joints(:,:);%calibration_data.robot_joints(ind,:);
% % initial_data.required_points  = calibration_data.required_points(:,:)*1e3;%calibration_data.required_points(ind,:)*1e3;
% % 
% % % Recompute flange pose based on robot joint states using direct
% % % kinematics
% % initial_data.robot_eef_pose2  = (directKinematics(initial_data.robot_parameters,initial_data))';
% % 
% % % Find rigid transformation between cameras and robot
% % [R,d,rms] = soder(initial_data.camera_eef_pose(:,1:3),initial_data.robot_eef_pose(:,1:3)); s = 1; % Rigid transformation
% % % [R,d,s,rms] = challis(initial_data.camera_eef_pose(:,1:3),initial_data.robot_eef_pose(:,1:3)); % Rigid transformation with uniform scaling 
% % % R = eye(3); d = zeros(3,1); s = 1; % No rigid transformation
% % Euler = rad2deg(R2mobileXYZ_array3(R));
% % 
% % % Store updated parameters
% % updated_data.camera_to_robot = [R d; 0 0 0 1];
% % updated_data.camera_eef_pose = (s*R*initial_data.camera_eef_pose(:,1:3)'+d)';
% % initial_data.camera_eef_pose = s*initial_data.camera_eef_pose; % Apply scale ratio to original data for 3D visualisation
% % % clear R d s rms;
% % 
% % Optimise robot geometrical parameters
% % x0      = initial_data.robot_parameters;
% % options = optimoptions(@fminunc,'PlotFcns',@optimplotfval,'Algorithm','quasi-newton','HessianApproximation','bfgs','StepTolerance',1e-10,'MaxFunctionEvaluations',1e6);
% % x       = fminunc(@(x)costFunction(x,initial_data,updated_data),x0,options);
% % x       = x0;
% % 
% % % Store updated parameters
% % updated_data.robot_parameters = x;
% % updated_data.robot_eef_pose   = directKinematics(x,initial_data)';
% % clear x x0 options;
% % 
% % % Error assessment
% % for t = 1:size(initial_data.robot_joints,1)
% %     error.camera.x0(t)   = sqrt((initial_data.robot_eef_pose(t,1) - initial_data.camera_eef_pose(t,1))^2);
% %     error.camera.y0(t)   = sqrt((initial_data.robot_eef_pose(t,2) - initial_data.camera_eef_pose(t,2))^2);
% %     error.camera.z0(t)   = sqrt((initial_data.robot_eef_pose(t,3) - initial_data.camera_eef_pose(t,3))^2);
% %     error.camera.e3D0(t) = sqrt(sum((initial_data.robot_eef_pose(t,1:3)-initial_data.camera_eef_pose(t,1:3)).^2));
% % 
% %     error.camera.x1(t)   = sqrt((updated_data.robot_eef_pose(t,1) - updated_data.camera_eef_pose(t,1))^2);
% %     error.camera.y1(t)   = sqrt((updated_data.robot_eef_pose(t,2) - updated_data.camera_eef_pose(t,2))^2);
% %     error.camera.z1(t)   = sqrt((updated_data.robot_eef_pose(t,3) - updated_data.camera_eef_pose(t,3))^2);
% %     error.camera.e3D1(t) = sqrt(sum((updated_data.robot_eef_pose(t,1:3)-updated_data.camera_eef_pose(t,1:3)).^2));
% % 
% %     error.target.x0(t)   = sqrt((initial_data.robot_eef_pose(t,1) - initial_data.required_points(t,1))^2);
% %     error.target.y0(t)   = sqrt((initial_data.robot_eef_pose(t,2) - initial_data.required_points(t,2))^2);
% %     error.target.z0(t)   = sqrt((initial_data.robot_eef_pose(t,3) - initial_data.required_points(t,3))^2);
% %     error.target.e3D0(t) = sqrt(sum((initial_data.robot_eef_pose(t,1:3)-initial_data.required_points(t,1:3)).^2));
% % 
% %     error.target.x1(t)   = sqrt((updated_data.robot_eef_pose(t,1) - initial_data.required_points(t,1))^2);
% %     error.target.y1(t)   = sqrt((updated_data.robot_eef_pose(t,2) - initial_data.required_points(t,2))^2);
% %     error.target.z1(t)   = sqrt((updated_data.robot_eef_pose(t,3) - initial_data.required_points(t,3))^2);
% %     error.target.e3D1(t) = sqrt(sum((updated_data.robot_eef_pose(t,1:3)-initial_data.required_points(t,1:3)).^2));
% % end
% % 
% % % Plot 3D distance error between target points
% % % kpoint = 1;
% % % for ipoint = 1:size(initial_data.required_points,1)-1
% % %     required_distance(kpoint) = sqrt(sum((initial_data.required_points(ipoint,1:3)-initial_data.required_points(ipoint+1,1:3)).^2));
% % %     required_robot(kpoint) = sqrt(sum((updated_data.robot_eef_pose(ipoint,1:3)-updated_data.robot_eef_pose(ipoint+1,1:3)).^2));
% % %     required_camera(kpoint) = sqrt(sum((updated_data.camera_eef_pose(ipoint,1:3)-updated_data.camera_eef_pose(ipoint+1,1:3)).^2));
% % %     kpoint = kpoint+1;
% % % end
% % % figure; hold on;
% % % bar([required_distance;required_robot;required_camera]');
% % 
% % % Plot 3D error
% % % figure; hold on; axis equal;
% % % title('3D error');
% % % quiver3(0,0,0,1,0,0,100,'red');
% % % quiver3(0,0,0,0,1,0,100,'green');
% % % quiver3(0,0,0,0,0,1,100,'blue');
% % % % for t = 1:size(initial_data.robot_joints,1)
% % % %     plot3(initial_data.required_points(t,1),initial_data.required_points(t,2),initial_data.required_points(t,3),'Linestyle','none','Marker','.','Markersize',20,'Color','black');
% % % %     plot3(initial_data.robot_eef_pose(t,1),initial_data.robot_eef_pose(t,2),initial_data.robot_eef_pose(t,3),'Linestyle','none','Marker','.','Markersize',20,'Color','red');
% % % %     plot3(initial_data.camera_eef_pose(t,1),initial_data.camera_eef_pose(t,2),initial_data.camera_eef_pose(t,3),'Linestyle','none','Marker','.','Markersize',20,'Color','blue');
% % % % end
% % % scatter3(initial_data.robot_eef_pose(:,1),initial_data.robot_eef_pose(:,2),initial_data.robot_eef_pose(:,3),[],error.camera.e3D1,'filled');
% % % colormap(gca,'default');
% % 
% % % Plot 2D error /camera
% % % figure;
% % % subplot(4,1,1); hold on; axis equal; box on; grid on;
% % % ylabel('Robot/camera error');
% % % yticks(1:20);
% % % yticklabels({'0','','2','','4','','6','','8','','10','','12','','14','','16','','18','','20'});
% % % ylim([0 20]);
% % % title('3D error');
% % % bar(1:n,[error.camera.e3D0; error.camera.e3D1]);
% % % subplot(4,1,2); hold on; axis equal; box on; grid on;
% % % ylabel('Robot/camera error');
% % % yticks(1:20);
% % % yticklabels({'0','','2','','4','','6','','8','','10','','12','','14','','16','','18','','20'});
% % % ylim([0 20]);
% % % title('X error');
% % % bar(1:n,[error.camera.x0; error.camera.x1]);
% % % subplot(4,1,3); hold on; axis equal; box on; grid on;
% % % ylabel('Robot/camera error');
% % % yticks(1:20);
% % % yticklabels({'0','','2','','4','','6','','8','','10','','12','','14','','16','','18','','20'});
% % % ylim([0 20]);
% % % title('Y error');
% % % bar(1:n,[error.camera.y0; error.camera.y1]);
% % % subplot(4,1,4); hold on; axis equal; box on; grid on;
% % % ylabel('Robot/camera error');
% % % yticks(1:20);
% % % yticklabels({'0','','2','','4','','6','','8','','10','','12','','14','','16','','18','','20'});
% % % ylim([0 20]);
% % % title('Z error');
% % % bar(1:n,[error.camera.z0; error.camera.z1]);
% % 
% % % Plot 2D error /target
% % % figure;
% % % subplot(4,1,1); hold on; axis equal; box on; grid on;
% % % ylabel('Robot/target error');
% % % yticks(1:20);
% % % yticklabels({'0','','2','','4','','6','','8','','10','','12','','14','','16','','18','','20'});
% % % ylim([0 20]);
% % % title('3D error');
% % % bar(1:n,[error.target.e3D0; error.target.e3D1]);
% % % subplot(4,1,2); hold on; axis equal; box on; grid on;
% % % ylabel('Robot/target error');
% % % yticks(1:20);
% % % yticklabels({'0','','2','','4','','6','','8','','10','','12','','14','','16','','18','','20'});
% % % ylim([0 20]);
% % % title('X error');
% % % bar(1:n,[error.target.x0; error.target.x1]);
% % % subplot(4,1,3); hold on; axis equal; box on; grid on;
% % % ylabel('Robot/target error');
% % % yticks(1:20);
% % % yticklabels({'0','','2','','4','','6','','8','','10','','12','','14','','16','','18','','20'});
% % % ylim([0 20]);
% % % title('Y error');
% % % bar(1:n,[error.target.y0; error.target.y1]);
% % % subplot(4,1,4); hold on; axis equal; box on; grid on;
% % % ylabel('Robot/target error');
% % % yticks(1:20);
% % % yticklabels({'0','','2','','4','','6','','8','','10','','12','','14','','16','','18','','20'});
% % % ylim([0 20]);
% % % title('Z error');
% % % bar(1:n,[error.target.z0; error.target.z1]);
% % 
% % % Plot differences updated camera position and robot parameters
% % % figure; hold on;
% % % plot(updated_data.camera_to_robot-initial_data.camera_to_robot,'blue');
% % % plot(updated_data.robot_parameters-initial_data.robot_parameters,'red');
% % 
% % close all;
% % figure;
% % subplot(1,2,1); hold on; ylim([0 10]);
% % plot(error.camera.x0(:),'red')
% % plot(error.camera.y0(:),'green')
% % plot(error.camera.z0(:),'blue')
% % subplot(1,2,2); hold on; ylim([0 10]);
% % plot(error.camera.x1(:),'red')
% % plot(error.camera.y1(:),'green')
% % plot(error.camera.z1(:),'blue')
% 
% % function f = costFunction(x,initial_data,required_points)
% %     % Set robot parameters
% %     f1             = [];
% %     f2             = [];
% %     robot_eef_pose = directKinematics(x,initial_data);
% %     % Frame-by-frame cost function
% %     for t = 1:size(initial_data.robot_joints,1)
% %         % Position
% %         Qr = robot_eef_pose(1:3,t); % Updated flange position in robot cartesian space
% %         Qc = (required_points(t,1:3))'; % Updated flange position in camera cartesian space 
% %         if ~isnan(sum(Qc))
% %             f1 = [f1 sqrt(sum((Qc-Qr).^2))];
% %         else
% %             f1 = [f1 0];
% %         end
% %         clear Qr Qc;
% % %         % Orientation
% % %         Qr = robot_eef_pose(4:6,t); % Updated flange orientation in robot cartesian space
% % %         Qc = -(required_points(t,4:6))'; % Updated flange orientation in camera cartesian space 
% % %         if ~isnan(sum(Qc)) && sqrt(sum((Qc-Qr).^2)) < 10 % Avoid Euler decomposition errors (assumed <10°)
% % %             f2 = [f2 sqrt(sum((Qc-Qr).^2))];
% % %         else
% % %             f2 = [f2 0];
% % %         end
% % %         clear Qr Qc;
% %     end
% %     % Global cost function
% %     f = sum(f1.^2)/size(initial_data.robot_joints,1);
% % %     f = sum(f1.^2)/size(initial_data.robot_joints,1); + ...
% % %         sum(f2.^2)/size(initial_data.robot_joints,1);
% % end
% % 
% % function robot_eef_pose = directKinematics(x,initial_data)
% %     % Set robot parameters
% %     x0    = initial_data.robot_parameters;
% %     d     = x(1:7);
% %     a     = x(8:14);
% %     theta = x(15:21);
% %     alfa  = x(22:28);
% %     for t = 1:size(initial_data.robot_joints,1)
% %         q = initial_data.robot_joints(t,:);
% %         % Compute direct kinematics
% %         T = zeros(4,4,7);
% %         i = 1;
% %         T(:,:,i) = getDHMatrix(alfa(i),q(i)+theta(i),d(i),a(i));
% %         for i = 2:7
% %             T(:,:,i) = T(:,:,i-1)*getDHMatrix(alfa(i),q(i)+theta(i),d(i),a(i));
% %             T(:,:,i) = normalizeColumns(T(:,:,i));
% %         end
% %         robot_eef_pose(1:3,t) = T(1:3,4,7);
% %         temp = rad2deg(R2mobileZYX_array3(T(1:3,1:3,7)));
% %         robot_eef_pose(4,t) = temp(3);
% %         robot_eef_pose(5,t) = temp(2);
% %         robot_eef_pose(6,t) = temp(1);
% %         clear temp;
% %     end
% % end
% % 
% % function T = getDHMatrix(alfa,theta,d,a)
% %     T        = zeros(4,4);    
% %     calpha   = cos(alfa);
% %     sinalpha = sin(alfa);
% %     coshteta = cos(theta);
% %     sintheta = sin(theta);    
% %     T(1,1)   = coshteta;
% %     T(2,1)   = sintheta*calpha;
% %     T(3,1)   = sintheta*sinalpha;    
% %     T(1,2)   = -sintheta;
% %     T(2,2)   = coshteta*calpha;
% %     T(3,2)   = coshteta*sinalpha;   
% %     T(2,3)   = -sinalpha;
% %     T(3,3)   = calpha;    
% %     T(1,4)   = a;
% %     T(2,4)   = -sinalpha*d;
% %     T(3,4)   = calpha*d;
% %     T(4,4)   = 1;
% % end
% % 
% % function normalizedT = normalizeColumns(T)
% %     %% This function is used to normalize the columns of a rotation matrix with
% %     % some numerical errors resulting from matrix multiplication problems
% %     r = zeros(4,3); % corrected rotation matrix, with zero badding row at the end
% %     for j = 1:3
% %         r(1:3,j) = T(1:3,j)/norm(T(1:3,j));
% %     end
% %     normalizedT = [r,T(:,4)];
% % end