% Author       : F. Moissenet
%                Biomechanics Laboratory (B-LAB)
%                University of Geneva
%                https://www.unige.ch/medecine/chiru/fr/b-lab-tests-robotises-avances-de-dispositifs-chirurgicaux/
% License      : Creative Commons Attribution-NonCommercial 4.0 International License 
%                https://creativecommons.org/licenses/by-nc/4.0/legalcode
% Source code  : https://github.com/fmoissenet/BLAB_Robohip_toolbox
% Reference    : To be defined
% Date         : September 2022
% -------------------------------------------------------------------------
% Description  : To be defined
% -------------------------------------------------------------------------
% Dependencies : To be defined
% -------------------------------------------------------------------------
% This work is licensed under the Creative Commons Attribution - 
% NonCommercial 4.0 International License. To view a copy of this license, 
% visit http://creativecommons.org/licenses/by-nc/4.0/ or send a letter to 
% Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
% -------------------------------------------------------------------------

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
Folder.toolbox = 'C:\Users\moissene\OneDrive - unige.ch\2022 - ROBOHIP\Développement\Biomécanique\BLAB_Robohip_toolbox\';
Folder.data    = 'C:\Users\moissene\OneDrive - unige.ch\2022 - ROBOHIP\Données\Test20220907\';
Folder.export  = 'C:\Users\moissene\OneDrive - unige.ch\2022 - ROBOHIP\Données\Test20220907\';
Folder.dep     = [Folder.toolbox,'dependencies\'];
addpath(Folder.toolbox);
addpath(genpath(Folder.dep));
cd(Folder.data);

% -------------------------------------------------------------------------
% SET SUBJECT INFO
% -------------------------------------------------------------------------
Subject.id   = 'RH000';
Subject.side = 'R'; % R or L
disp(['Subject ID: ',Subject.id]);
disp(['Assessed side: ',Subject.side]);

% -------------------------------------------------------------------------
% SET ROBOT PARAMETERS
% -------------------------------------------------------------------------
Robot.heightWorkspaceCentre = 360; % Robot workspace height centre (mm)
Robot.minRadius             = 420; % Robot workspace inferior boundary (mm)
Robot.maxRadius             = Robot.minRadius + 400; % Robot workspace superior boundary (mm)
Robot.lenghtFlange          = 151.4; % Electrical touch flange length (mm)
Robot.lenghtAssembly        = 15; % Height of the assembly on the flange (mm)
Robot.cutMargin             = 10; % Humerus cut positioning margin (mm)
Robot.tractionMargin        = 5; % Humerus traction positioning margin (mm)
Robot.robotMargin           = 20; % Robot related additional positioning margin (mm)
Robot.lenghtSupporthumerus  = 31; % Length of the humerus support (mm)
Robot.DiamSupporthumerus    = 63; % Diameter of the humerus support (mm)
Robot.ThicknessBotaPlate    = 10; % Thickness of the Bota sensor plate (mm)
Robot.ThicknessBotaSensor   = 44.5; % Thickness of the Bota sensor (mm)
Robot.ThicknessToolPlate    = 10; % Thickness of the connection tool plate (mm)

% %% ------------------------------------------------------------------------
% % DEFINE THE RIGID TRANSFORMATION BETWEEN MOCAP ICS AND ROBOT BASE
% % -------------------------------------------------------------------------
% 
% % Get real marker positions stored in C3D files
% % -------------------------------------------------------------------------
% btkFile      = btkReadAcquisition('robotCalibration01.c3d');
% Marker       = btkGetMarkers(btkFile);
% markerNames  = fieldnames(Marker); % Marker names in the marker trajectories
% nMarker      = length(Marker.BAS01); % n frames stored in the marker trajectories
% fMarker      = btkGetPointFrequency(btkFile); % Marker trajectories frequency
% dMarker      = 14; % mm
% 
% % Compute mean position across all frames
% % -------------------------------------------------------------------------
% Marker.BAS01 = mean(Marker.BAS01,1); % mm
% Marker.BAS02 = mean(Marker.BAS02,1); % mm
% Marker.BAS03 = mean(Marker.BAS03,1); % mm
% Marker.BAS04 = mean(Marker.BAS04,1); % mm
% 
% % Set virtual marker positions based on geometric assumptions
% % -------------------------------------------------------------------------
% Xbase         = [1 0 0];
% Ybase         = [0 1 0];
% Zbase         = [0 0 1];
% Obase         = [0.0 0.0 0.0]; % Based on technical documentation (mm)
% Vmarker.BAS01 = Obase ...
%                 + 340/2*Xbase ... % Defined on CAO documentation fromm HEPIA (mm)
%                 - 220/2*Ybase ... % Defined on CAO documentation fromm HEPIA (mm)
%                 - 1*Zbase; % Depth of the marker base placement on the flange plate (mm)
%                 + dMarker/2*Zbase;
% Vmarker.BAS02 = Obase ...
%                 - 340/2*Xbase ... % Defined on CAO documentation fromm HEPIA (mm)
%                 - 220/2*Ybase ... % Defined on CAO documentation fromm HEPIA (mm)
%                 - 1*Zbase; % Depth of the marker base placement on the flange plate (mm)
%                 + dMarker/2*Zbase;
% Vmarker.BAS03 = Obase ...
%                 - 340/2*Xbase ... % Defined on CAO documentation fromm HEPIA (mm)
%                 + 220/2*Ybase ... % Defined on CAO documentation fromm HEPIA (mm)
%                 - 1*Zbase; % Depth of the marker base placement on the flange plate (mm)
%                 + dMarker/2*Zbase;
% Vmarker.BAS04 = Obase ...
%                 + 340/2*Xbase ... % Defined on CAO documentation fromm HEPIA (mm)
%                 + 220/2*Ybase ... % Defined on CAO documentation fromm HEPIA (mm)
%                 - 1*Zbase; % Depth of the marker base placement on the flange plate (mm)
%                 + dMarker/2*Zbase;
% 
% % Compute the rigid transformation between coordinate systems
% % -------------------------------------------------------------------------
% [R_qualisys_robot,d_qualisys_robot,rms] = soder([Marker.BAS01;Marker.BAS02;Marker.BAS03;Marker.BAS04], ...
%                                                 [Vmarker.BAS01;Vmarker.BAS02;Vmarker.BAS03;Vmarker.BAS04]);
% 
% % Plot (FOR TEST ONLY)
% % -------------------------------------------------------------------------
% % Marker.BAS01s = (R_qualisys_robot*Marker.BAS01'+d_qualisys_robot)';
% % Marker.BAS02s = (R_qualisys_robot*Marker.BAS02'+d_qualisys_robot)';
% % Marker.BAS03s = (R_qualisys_robot*Marker.BAS03'+d_qualisys_robot)';
% % Marker.BAS04s = (R_qualisys_robot*Marker.BAS04'+d_qualisys_robot)';
% % figure;
% % hold on;
% % axis equal;
% % plot3(Marker.BAS01(1),Marker.BAS01(2),Marker.BAS01(3),'Marker','x','Color','black');
% % plot3(Marker.BAS02(1),Marker.BAS02(2),Marker.BAS02(3),'Marker','x','Color','red');
% % plot3(Marker.BAS03(1),Marker.BAS03(2),Marker.BAS03(3),'Marker','x','Color','red');
% % plot3(Marker.BAS04(1),Marker.BAS04(2),Marker.BAS04(3),'Marker','x','Color','red');
% % plot3(Vmarker.BAS01(1),Vmarker.BAS01(2),Vmarker.BAS01(3),'Marker','o','Color','blue');
% % plot3(Vmarker.BAS02(1),Vmarker.BAS02(2),Vmarker.BAS02(3),'Marker','o','Color','blue');
% % plot3(Vmarker.BAS03(1),Vmarker.BAS03(2),Vmarker.BAS03(3),'Marker','o','Color','blue');
% % plot3(Vmarker.BAS04(1),Vmarker.BAS04(2),Vmarker.BAS04(3),'Marker','o','Color','blue');
% % plot3(Marker.BAS01s(1),Marker.BAS01s(2),Marker.BAS01s(3),'Marker','x','Color','black');
% % plot3(Marker.BAS02s(1),Marker.BAS02s(2),Marker.BAS02s(3),'Marker','x','Color','green');
% % plot3(Marker.BAS03s(1),Marker.BAS03s(2),Marker.BAS03s(3),'Marker','x','Color','green');
% % plot3(Marker.BAS04s(1),Marker.BAS04s(2),Marker.BAS04s(3),'Marker','x','Color','green');
% 
% % Clear workspace
% % -------------------------------------------------------------------------
% clearvars -except Folder Subject Robot R_qualisys_robot d_qualisys_robot;
% 
% %% ------------------------------------------------------------------------
% % APPLY THE RIGID TRANSFORMATION BETWEEN MOCAP ICS AND ROBOT BASE AND
% % ASSESS THE ERROR BETWEEN ROBOT POSITIONS AND MOCAP POSITIONS
% % -------------------------------------------------------------------------
% d_error = nan(3,1); % No calibration at this stage, should be defined if needed in the future

%% ------------------------------------------------------------------------
% SET THE TIP LOCATION OF THE STYLUS
% -------------------------------------------------------------------------

% Load marker trajectories stored in C3D files
% -------------------------------------------------------------------------
btkFile     = btkReadAcquisition('stylusCalibration01.c3d');
Marker      = btkGetMarkers(btkFile);
markerNames = fieldnames(Marker); % Marker names in the marker trajectories
nMarker     = length(Marker.STY01); % n frames stored in the marker trajectories
fMarker     = btkGetPointFrequency(btkFile); % Marker trajectories frequency

% Define stylus coordinate system
% -------------------------------------------------------------------------
Os      = Marker.STY04;
Ys      = (Marker.STY02-Marker.STY05);
Ys      = Ys./sqrt(Ys(:,1).^2+Ys(:,2).^2+Ys(:,3).^2);
Xs      = cross((Marker.STY03-Marker.STY05),(Marker.STY01-Marker.STY05));
Xs      = Xs./sqrt(Xs(:,1).^2+Xs(:,2).^2+Xs(:,3).^2);   
Zs      = cross(Xs,Ys);
for i = 1:nMarker
    T_ics_s(i,:,:) = [[Xs(i,:)' Ys(i,:)' Zs(i,:)'] Os(i,:)'; 0 0 0 1]; % from s to ics
end

% Compute the averaged centre of rotation between T_ics_s and T_ics_ics
% -------------------------------------------------------------------------
T_ics_ics               = repmat(permute([[1 0 0; 0 1 0; 0 0 1] [0 0 0]'; 0 0 0 1],[3,1,2]),[nMarker 1 1]); % from ics to ics
[rC,rCsi,rCsj,Residual] = SCoRE_array3(permute(T_ics_s,[2,3,1]),permute(T_ics_ics,[2,3,1]));
STY06 = rCsi; % local position

% Clear workspace
% -------------------------------------------------------------------------
clearvars -except Folder Subject Robot R_qualisys_robot d_qualisys_robot d_error STY06;

%% ------------------------------------------------------------------------
% DEFINE VIRTUAL MARKER AT BONY LANDMARKS
% Position of virtual markers are stored in the local technical coordinate
% system defined using the local marker cluster
% -------------------------------------------------------------------------

% Before femur cut
% -------------------------------------------------------------------------
% Load bone calibration file
btkFile     = btkReadAcquisition('boneCalibration01.c3d');
Marker      = btkGetMarkers(btkFile);
markerNames = fieldnames(Marker); % Marker names in the marker trajectories
nMarker     = length(Marker.STY01); % n frames stored in the marker trajectories
fMarker     = btkGetPointFrequency(btkFile); % Marker trajectories frequency

% Compute mean position across all frames (except Stylus markers)
% LIMB MUST NOT MOVE DURING RECORDING !
for imarker = 1:size(markerNames,1)
    if isempty(strfind(markerNames{imarker},'STY'))
        Marker.(markerNames{imarker}) = mean(Marker.(markerNames{imarker}),1);
    end
end

% Define stylus coordinate system
Os = Marker.STY04;
Ys = (Marker.STY02-Marker.STY05);
Ys = Ys./sqrt(Ys(:,1).^2+Ys(:,2).^2+Ys(:,3).^2);
Xs = cross((Marker.STY03-Marker.STY05),(Marker.STY01-Marker.STY05));
Xs = Xs./sqrt(Xs(:,1).^2+Xs(:,2).^2+Xs(:,3).^2);   
Zs = cross(Xs,Ys);
for i = 1:nMarker
    T_ics_s(i,:,:) = [[Xs(i,:)' Ys(i,:)' Zs(i,:)'] Os(i,:)'; 0 0 0 1]; % from s to ics
end

% Compute the virtual marker corresponding to the stylus tip
for i = 1:nMarker
    temp                = permute(T_ics_s(i,:,:),[2,3,1])*[STY06;1];
    Marker.STY06(i,:,:) = temp(1:3);
end

% Get the global position of the tip at key events
% SAME EVENTS DEFINED IN QTM
% landmarks   = {'RIAS' 'LIAS' 'RIPS' 'LIPS' 'RFLE' 'RFME' 'LFLE' 'LFME'};
landmarks   = {'RIAS' 'LIAS' 'RIPS' 'LIPS' 'LFLE' 'LFME'};
Event       = btkGetEvents(btkFile);
for ilandmark = 1:size(landmarks,2)
    Marker.(landmarks{ilandmark}) = Marker.STY06(round(Event.(landmarks{ilandmark})*fMarker)+1-btkGetFirstFrame(btkFile)+1,:);
end
markerNames = fieldnames(Marker);

% Extract landmarks and cluster markers
Calibration.Pelvis.cluster   = [permute(Marker.cPEL01,[2,3,1]) ...
                                permute(Marker.cPEL02,[2,3,1]) ...
                                permute(Marker.cPEL03,[2,3,1]) ...
                                permute(Marker.cPEL04,[2,3,1])];
Calibration.Pelvis.landmarks = [permute(Marker.RIAS,[2,3,1]) ...
                                permute(Marker.LIAS,[2,3,1]) ...
                                permute(Marker.RIPS,[2,3,1]) ...
                                permute(Marker.LIPS,[2,3,1])];
% Calibration.RFemur.cluster   = [permute(Marker.cRFEM01,[2,3,1]) ...
%                                 permute(Marker.cRFEM02,[2,3,1]) ...
%                                 permute(Marker.cRFEM03,[2,3,1]) ...
%                                 permute(Marker.cRFEM04,[2,3,1])];
% Calibration.RFemur.landmarks = [permute(Marker.RFLE,[2,3,1]) ...
%                                 permute(Marker.RFME,[2,3,1])];
Calibration.LFemur.cluster   = [permute(Marker.cLFEM01,[2,3,1]) ...
                                permute(Marker.cLFEM02,[2,3,1]) ...
                                permute(Marker.cLFEM03,[2,3,1]) ...
                                permute(Marker.cLFEM04,[2,3,1])];
Calibration.LFemur.landmarks = [permute(Marker.LFLE,[2,3,1]) ...
                                permute(Marker.LFME,[2,3,1])];

% Clear workspace
clearvars -except Folder Subject Robot R_qualisys_robot d_qualisys_robot d_error STY06 Calibration;

figure;
hold on; axis equal;
plot3(Calibration.Pelvis.cluster(1,:),...
      Calibration.Pelvis.cluster(2,:),...
      Calibration.Pelvis.cluster(3,:),...
      'Marker','.','Markersize',15,'Linestyle','none','Color','red');
plot3(Calibration.Pelvis.landmarks(1,:),...
      Calibration.Pelvis.landmarks(2,:),...
      Calibration.Pelvis.landmarks(3,:),...
      'Marker','.','Markersize',15,'Linestyle','none','Color','green');
plot3(Calibration.LFemur.cluster(1,:),...
      Calibration.LFemur.cluster(2,:),...
      Calibration.LFemur.cluster(3,:),...
      'Marker','.','Markersize',15,'Linestyle','none','Color','red');
plot3(Calibration.LFemur.landmarks(1,:),...
      Calibration.LFemur.landmarks(2,:),...
      Calibration.LFemur.landmarks(3,:),...
      'Marker','.','Markersize',15,'Linestyle','none','Color','green');


% Load bone functional file
% One file with both right and left femur functional task inside
btkFile     = btkReadAcquisition('boneFunctional01.c3d');
Marker      = btkGetMarkers(btkFile);
markerNames = fieldnames(Marker); % Marker names in the marker trajectories
nMarker     = length(Marker.cPEL01); % n frames stored in the marker trajectories
fMarker     = btkGetPointFrequency(btkFile); % Marker trajectories frequency

% Extract cluster markers
Functional.Pelvis.cluster = [permute(Marker.cPEL01,[2,3,1]) ...
                             permute(Marker.cPEL02,[2,3,1]) ...
                             permute(Marker.cPEL03,[2,3,1]) ...
                             permute(Marker.cPEL04,[2,3,1])];
% Functional.RFemur.cluster = [permute(Marker.cRFEM01,[2,3,1]) ...
%                              permute(Marker.cRFEM02,[2,3,1]) ...
%                              permute(Marker.cRFEM03,[2,3,1]) ...
%                              permute(Marker.cRFEM04,[2,3,1])];
Functional.LFemur.cluster = [permute(Marker.cLFEM01,[2,3,1]) ...
                             permute(Marker.cLFEM02,[2,3,1]) ...
                             permute(Marker.cLFEM03,[2,3,1]) ...
                             permute(Marker.cLFEM04,[2,3,1])];

% Compute landmarks
for t = 1:nMarker
    [R,d,rms] = soder(Calibration.Pelvis.cluster',Functional.Pelvis.cluster(:,:,t)');
    Functional.Pelvis.landmarks(:,:,t) = R*Calibration.Pelvis.landmarks+d;
    clear R d rms;
%     [Rr(:,:,t),dr(:,:,t),rms] = soder(Calibration.RFemur.cluster',Functional.RFemur.cluster(:,:,t)');
%     Functional.RFemur.landmarks(:,:,t) = Rr(:,:,t)*Calibration.RFemur.landmarks+dr(:,:,t);
%     clear rms;
    [Rl(:,:,t),dl(:,:,t),rms] = soder(Calibration.LFemur.cluster',Functional.LFemur.cluster(:,:,t)');
    Functional.LFemur.landmarks(:,:,t) = Rl(:,:,t)*Calibration.LFemur.landmarks+dl(:,:,t);
    clear rms;
end

% Set technical homogeneous matrices
Xp = Vnorm_array3(Functional.Pelvis.cluster(:,2,:)-Functional.Pelvis.cluster(:,1,:));
Zp = Vnorm_array3(cross(Functional.Pelvis.cluster(:,3,:)-Functional.Pelvis.cluster(:,4,:), ...
                        Functional.Pelvis.cluster(:,1,:)-Functional.Pelvis.cluster(:,4,:)));
Yp = Vnorm_array3(cross(Zp,Xp));
Rp = [Xp Yp Zp];
Op = Functional.Pelvis.cluster(:,1,:);
Tp = [Rp Op; repmat([0 0 0 1],[1,1,nMarker])];
% --
% Xrf = Vnorm_array3(Functional.RFemur.cluster(:,2,:)-Functional.RFemur.cluster(:,1,:));
% Zrf = Vnorm_array3(cross(Functional.RFemur.cluster(:,3,:)-Functional.RFemur.cluster(:,4,:), ...
%                          Functional.RFemur.cluster(:,1,:)-Functional.RFemur.cluster(:,4,:)));
% Yrf = Vnorm_array3(cross(Zrf,Xrf));
% Rrf = [Xrf Yrf Zrf];
% Orf = Functional.RFemur.cluster(:,1,:);
% Trf = [Rrf Orf; repmat([0 0 0 1],[1,1,nMarker])];
% --
Xlf = Vnorm_array3(Functional.LFemur.cluster(:,2,:)-Functional.LFemur.cluster(:,1,:));
Zlf = Vnorm_array3(cross(Functional.LFemur.cluster(:,3,:)-Functional.LFemur.cluster(:,4,:), ...
                         Functional.LFemur.cluster(:,1,:)-Functional.LFemur.cluster(:,4,:)));
Ylf = Vnorm_array3(cross(Zlf,Xlf));
Rlf = [Xlf Ylf Zlf];
Olf = Functional.LFemur.cluster(:,1,:);
Tlf = [Rlf Olf; repmat([0 0 0 1],[1,1,nMarker])];

% Compute instantaneous centre of rotations
% [rC,rCsi,rCsj,Residual]      = SCoRE_array3(Trf,Tp);
% Functional.RFemur.landmarks  = [Functional.RFemur.landmarks rC];
% clear rC rCsi rCsj Residual;
[rC,rCsi,rCsj,Residual]      = SCoRE_array3(Tlf,Tp);
Functional.LFemur.landmarks  = [Functional.LFemur.landmarks rC];
clear rC rCsi rCsj Residual;
clear Rr dr Rl dl;

figure;
hold on; axis equal;
t = 1;
plot3(Functional.Pelvis.cluster(1,:,t),...
      Functional.Pelvis.cluster(2,:,t),...
      Functional.Pelvis.cluster(3,:,t),...
      'Marker','.','Markersize',15,'Linestyle','none','Color','red');
plot3(Functional.Pelvis.landmarks(1,:,t),...
      Functional.Pelvis.landmarks(2,:,t),...
      Functional.Pelvis.landmarks(3,:,t),...
      'Marker','.','Markersize',15,'Linestyle','none','Color','green');
plot3(Functional.LFemur.cluster(1,:,t),...
      Functional.LFemur.cluster(2,:,t),...
      Functional.LFemur.cluster(3,:,t),...
      'Marker','.','Markersize',15,'Linestyle','none','Color','red');
plot3(Functional.LFemur.landmarks(1,:,t),...
      Functional.LFemur.landmarks(2,:,t),...
      Functional.LFemur.landmarks(3,:,t),...
      'Marker','.','Markersize',15,'Linestyle','none','Color','green');
plot3(Functional.LFemur.landmarks(1,end,t),...
      Functional.LFemur.landmarks(2,end,t),...
      Functional.LFemur.landmarks(3,end,t),...
      'Marker','.','Markersize',15,'Linestyle','none','Color','blue');

% Clear workspace
clearvars -except Folder Subject Robot R_qualisys_robot d_qualisys_robot d_error STY06 Calibration Functional;

% After femur cut
% -------------------------------------------------------------------------
% Load static file
% THE LIMB IS ALREADY CONNECTED TO THE ROBOT AT THIS STAGE
btkFile     = btkReadAcquisition('boneStatic01.c3d');
Marker      = btkGetMarkers(btkFile);
markerNames = fieldnames(Marker); % Marker names in the marker trajectories
nMarker     = length(Marker.cPEL01); % n frames stored in the marker trajectories
fMarker     = btkGetPointFrequency(btkFile); % Marker trajectories frequency (Hz)
dMarker     = 9.5; % Diameter of the markers placed on the flange (mm)

% Compute mean position across all frames
% LIMB MUST NOT MOVE DURING RECORDING !
for imarker = 1:size(markerNames,1)
    Marker.(markerNames{imarker}) = mean(Marker.(markerNames{imarker}),1);
end

% Extract cluster markers
Static.Pelvis.cluster = [permute(Marker.cPEL01,[2,3,1]) ...
                         permute(Marker.cPEL02,[2,3,1]) ...
                         permute(Marker.cPEL03,[2,3,1]) ...
                         permute(Marker.cPEL04,[2,3,1])];
% Static.RFemur.cluster = [permute(Marker.cRFEM01,[2,3,1]) ...
%                          permute(Marker.cRFEM02,[2,3,1]) ...
%                          permute(Marker.cRFEM03,[2,3,1]) ...
%                          permute(Marker.cRFEM04,[2,3,1])];
Static.LFemur.cluster = [permute(Marker.cLFEM01,[2,3,1]) ...
                         permute(Marker.cLFEM02,[2,3,1]) ...
                         permute(Marker.cLFEM03,[2,3,1]) ...
                         permute(Marker.cLFEM04,[2,3,1])];

% Compute landmarks in ICS
[R,d,rms] = soder(Functional.Pelvis.cluster(:,:,1)',Static.Pelvis.cluster');
Static.Pelvis.landmarks = R*Functional.Pelvis.landmarks(:,:,1)+d;
clear R d rms;
%     [R,d,rms] = soder(Functional.RFemur.cluster(:,:,1)',Static.RFemur.cluster');
%     Static.RFemur.landmarks = R*Functional.RFemur.landmarks(:,:,1)+d;
%     clear R d rms;
[R,d,rms] = soder(Functional.LFemur.cluster(:,:,1)',Static.LFemur.cluster');
Static.LFemur.landmarks = R*Functional.LFemur.landmarks(:,:,1)+d;
clear R d rms;

% FOR TEST ONLY
Zlf = Vnorm_array3(Static.LFemur.landmarks(:,1)-Static.LFemur.landmarks(:,2));
Ylf = Vnorm_array3(Static.LFemur.landmarks(:,3)-(Static.LFemur.landmarks(:,1)+Static.LFemur.landmarks(:,2))/2);
Xlf = Vnorm_array3(cross(Ylf,Zlf));
Marker.END01 = permute((Static.LFemur.landmarks(:,1)+Static.LFemur.landmarks(:,2))/2 + 150*Ylf + (83/2+dMarker/2)*Xlf,[3,1,2]);
Marker.END02 = permute((Static.LFemur.landmarks(:,1)+Static.LFemur.landmarks(:,2))/2 + 150*Ylf - (83/2+dMarker/2)*Zlf,[3,1,2]);
Marker.END03 = permute((Static.LFemur.landmarks(:,1)+Static.LFemur.landmarks(:,2))/2 + 150*Ylf - (83/2+dMarker/2)*Xlf,[3,1,2]);
Marker.END04 = permute((Static.LFemur.landmarks(:,1)+Static.LFemur.landmarks(:,2))/2 + 150*Ylf + (83/2+dMarker/2)*Zlf,[3,1,2]);

% Define robot flange markers (also considered as femur landmarks)
Static.LFemur.landmarks = [Static.LFemur.landmarks ...
                           permute(Marker.END01,[2,3,1]) ...
                           permute(Marker.END02,[2,3,1]) ...
                           permute(Marker.END03,[2,3,1]) ...
                           permute(Marker.END04,[2,3,1])];

figure;
hold on; axis equal;
t = 1;
plot3(Static.Pelvis.cluster(1,:,t),...
      Static.Pelvis.cluster(2,:,t),...
      Static.Pelvis.cluster(3,:,t),...
      'Marker','.','Markersize',15,'Linestyle','none','Color','red');
plot3(Static.Pelvis.landmarks(1,:,t),...
      Static.Pelvis.landmarks(2,:,t),...
      Static.Pelvis.landmarks(3,:,t),...
      'Marker','.','Markersize',15,'Linestyle','none','Color','green');
plot3(Static.LFemur.cluster(1,:,t),...
      Static.LFemur.cluster(2,:,t),...
      Static.LFemur.cluster(3,:,t),...
      'Marker','.','Markersize',15,'Linestyle','none','Color','red');
plot3(Static.LFemur.landmarks(1,:,t),...
      Static.LFemur.landmarks(2,:,t),...
      Static.LFemur.landmarks(3,:,t),...
      'Marker','.','Markersize',15,'Linestyle','none','Color','green');

% Define hip joint coordinate system (JCS) at reference posture (all angles = 0°)
e1 = Vnorm_array3(Static.Pelvis.landmarks(:,1)-Static.Pelvis.landmarks(:,2)); % Flexion/extension axis
e3 = Vnorm_array3(Static.LFemur.landmarks(:,3)-(Static.LFemur.landmarks(:,1)+Static.LFemur.landmarks(:,2))/2); % Internal/external axis
e2 = Vnorm_array3(cross(e3,e1));
T_ics_hipref = [e1 e2 e3 Static.LFemur.landmarks(:,3); 0 0 0 1];

% Compute femur landmarks in hip JCS
temp = Mprod_array3(Minv_array3(T_ics_hipref),[Static.LFemur.landmarks; ones(1,size(Static.LFemur.landmarks,2),1)])
Static.LFemur.localLandmarks = temp(1:3,:,:);
clear temp;

% Create a motion: 30° flexion around hip JCS flexion/extension axis
angles  = 0:1:30; % (deg)
% /e1
% Tmotion = [ones(1,1,length(angles))       zeros(1,1,length(angles))      zeros(1,1,length(angles))      zeros(1,1,length(angles)); ...
%               zeros(1,1,length(angles))      permute( cosd(angles),[1,3,2]) permute(-sind(angles),[1,3,2]) zeros(1,1,length(angles)); ...
%               zeros(1,1,length(angles))      permute( sind(angles),[1,3,2]) permute( cosd(angles),[1,3,2]) zeros(1,1,length(angles)); ...
%               zeros(1,1,length(angles))      zeros(1,1,length(angles))      zeros(1,1,length(angles))      ones(1,1,length(angles))];
% /e2
Tmotion = [permute( cosd(angles),[1,3,2]) zeros(1,1,length(angles))      permute( sind(angles),[1,3,2]) zeros(1,1,length(angles)); ...
              zeros(1,1,length(angles))      ones(1,1,length(angles))       zeros(1,1,length(angles))      zeros(1,1,length(angles)); ...
              permute(-sind(angles),[1,3,2]) zeros(1,1,length(angles))      permute( cosd(angles),[1,3,2]) zeros(1,1,length(angles)); ...
              zeros(1,1,length(angles))      zeros(1,1,length(angles))      zeros(1,1,length(angles))      ones(1,1,length(angles))];
% /e3
% Tmotion = [permute( cosd(angles),[1,3,2]) permute(-sind(angles),[1,3,2]) zeros(1,1,length(angles))      zeros(1,1,length(angles)); ...
%               permute( sind(angles),[1,3,2]) permute( cosd(angles),[1,3,2]) zeros(1,1,length(angles))      zeros(1,1,length(angles)); ...
%               zeros(1,1,length(angles))      zeros(1,1,length(angles))      ones(1,1,length(angles))       zeros(1,1,length(angles)); ...
%               zeros(1,1,length(angles))      zeros(1,1,length(angles))      zeros(1,1,length(angles))      ones(1,1,length(angles))];
temp = Mprod_array3(repmat(T_ics_hipref,[1 1 length(angles)]),...
                    Mprod_array3(Tmotion,...
                                 repmat([Static.LFemur.localLandmarks; ones(1,size(Static.LFemur.landmarks,2),1)],[1 1 length(angles)])));
Static.LFemur.landmarks2 = temp(1:3,:,:);
clear temp;

% Update T_ics_hipref before creating a new motion
% Generate an internal rotation motion

% Flexion 0° - IR 90° ?
% Flexion 0° - Return
% Flexion 0° - ER 90° ?
% ... avec flexion +/- 30°, 60°, 90° ?

% Préparer un code qui me permette de facilement modifier les trajectoires
% requises en fonction de ce qu'on va voir/mesurer pendant les essais
% ATTENTION A NE PAS DETRUIRE LA PIECE !!

close all;
figure;
for t = 1:length(angles)
    plot3(Static.Pelvis.cluster(1,:,1),...
          Static.Pelvis.cluster(2,:,1),...
          Static.Pelvis.cluster(3,:,1),...
          'Marker','.','Markersize',15,'Linestyle','none','Color','red');
    hold on; axis equal;
    plot3(Static.Pelvis.landmarks(1,:,1),...
          Static.Pelvis.landmarks(2,:,1),...
          Static.Pelvis.landmarks(3,:,1),...
          'Marker','.','Markersize',15,'Linestyle','none','Color','green');
    plot3(Static.LFemur.cluster(1,:,1),...
          Static.LFemur.cluster(2,:,1),...
          Static.LFemur.cluster(3,:,1),...
          'Marker','.','Markersize',15,'Linestyle','none','Color','red');
    plot3(Static.LFemur.landmarks(1,:,1),...
          Static.LFemur.landmarks(2,:,1),...
          Static.LFemur.landmarks(3,:,1),...
          'Marker','.','Markersize',15,'Linestyle','none','Color','green');
    plot3(Static.LFemur.landmarks2(1,:,t),...
          Static.LFemur.landmarks2(2,:,t),...
          Static.LFemur.landmarks2(3,:,t),...
          'Marker','.','Markersize',15,'Linestyle','none','Color','blue');
    if t < length(angles)
        pause(0.001);
        cla;
        hold off;
    end
end



% AVANT COUPE FEMUR
% Charger positions statique marqueur cluster femur
% Charger pointages statiques des landmarks ISB femur
% Fonctionnelle hanche
% Définir le système d'axes hanche

% APRES COUPE FEMUR
% Charger positions statique des marqueurs qualifiant la flange
% Définir le système d'axes flange

% Calculer la position actualisée des marqueurs clusters en appliquant une
% rotation autour d'un axe unique
% Calculer (soder) la position actualisée des landmarks ?
% Calculer la position actualisée du système d'axes flange
% Définir le quaternion flange pour le robot


% Set the body segment and joint coordinate systems
% -------------------------------------------------------------------------

% Static flange coordinate system XfsYfsZfs
% Not ISB
Zfs = Marker.END01-Marker.END03;
Zfs = Zfs./sqrt(Zfs(:,1).^2+Zfs(:,2).^2+Zfs(:,3).^2);
Yfs = cross(Marker.END01-Marker.END04,Marker.END03-Marker.END04);
Yfs = Yfs./sqrt(Yfs(:,1).^2+Yfs(:,2).^2+Yfs(:,3).^2);
Xfs = -cross(Zfs,Yfs);
Zfs = cross(Xfs,Yfs);
Ofs = (Marker.END01+Marker.END02+Marker.END03+Marker.END04)/4 ...
       + dMarker/2*Yfs ...
       - 1*Yfs; % Depth of the marker base placement on the flange plate (mm)
T_ics_fs = [[Xfs' Yfs' Zfs'] Ofs'; 0 0 0 1]; % from fs to ics

% Static ulnar coordinate system XuYuZu (defined with forearm in the 
% neutral position and and elbow flexed 90° in the sagittal plane)
% Wu et al. 2005
Ous      = Marker.US(:,:); % Most caudal-medial point on the ulnar process
Yus      = (Marker.EM(:,:)+Marker.EL(:,:))/2 - Marker.US(:,:);
Yus      = Yus/sqrt(Yus(:,1).^2+Yus(:,2).^2+Yus(:,3).^2);
Xus      = cross(Marker.RS(:,:)-(Marker.EL(:,:)+Marker.EM(:,:))/2, ...
                 Marker.US(:,:)-(Marker.EL(:,:)+Marker.EM(:,:))/2);
Xus      = Xus/sqrt(Xus(:,1).^2+Xus(:,2).^2+Xus(:,3).^2);
Zus      = cross(Xus,Yus);

% Static radius coordinate system XrYrZr (defined with forearm in the 
% neutral position and and elbow flexed 90° in the sagittal plane)
% Wu et al. 2005    
Ors      = Marker.RS(:,:); % Most caudal-lateral point on the radial process
Yrs      = Marker.EL(:,:)-Marker.RS(:,:);
Yrs      = Yrs/sqrt(Yrs(:,1).^2+Yrs(:,2).^2+Yrs(:,3).^2);
Xrs      = cross(Marker.RS(:,:)-Marker.EL(:,:), ...
                 Marker.US(:,:)-Marker.EL(:,:));
Xrs      = Xrs/sqrt(Xrs(:,1).^2+Xrs(:,2).^2+Xrs(:,3).^2);
Zrs      = cross(Xrs,Yrs);
    
% Static radioulnar intermediate coordinate system XriYriZri (defined with 
% forearm in the neutral position and and elbow flexed 90° in the sagittal plane)
% Wu et al. 2005   
Oris      = Ors;
Xris      = Xus; % Radial/ulnar deviation axis
Yris      = Yus; % Pronosupination axis
Zris      = Zus; % Flexion/extension axis
T_ics_ris = [[Xris' Yris' Zris'] Oris'; 0 0 0 1]; % from ris to ics
T_ics_ri  = T_ics_ris; % ris attached to ulna assumed to be rigidly connected with ground

% Define the rigid transformation between static hand and static flange
T_fs_ris = inv(T_ics_fs)*T_ics_ris; % from ris to fs
T_f_ri   = T_fs_ris; % from ris to fs

%% ------------------------------------------------------------------------
% DEFINE THE REQUESTED MOTIONS
% -------------------------------------------------------------------------

% Set the initial movement aligning the flange to the radioulnar
% intermediate coordinate system
% -------------------------------------------------------------------------

% Initialise the dynamic frange coordinate system XfYfZf
Xf = [];
Yf = [];
Zf = [];
Of = [];
    
% Align the flange axes and ris axis using 3 consecutive rotations / 100 frames
% The transformation to align fs on ris is: T_ics_fs*[T_fs_ris(1:3,1:3) [0 0 0]'; 0 0 0 1];
% The related Euler angles are: R2fixedZYX_array3(T_fs_ris(1:3,1:3));
% The resulting rotation matrix is: [cos(Euler(1)) -sin(Euler(1)) 0; sin(Euler(1)) cos(Euler(1)) 0; 0 0 1]*...
%                                   [cos(Euler(2)) 0 sin(Euler(2)); 0 1 0; -sin(Euler(2)) 0 cos(Euler(2))]*...
%                                   [1 0 0; 0 cos(Euler(3)) -sin(Euler(3)); 0 sin(Euler(3)) cos(Euler(3))];
nframe = 200;
Euler  = R2fixedZYX_array3(T_fs_ris(1:3,1:3));
if Euler(1) > 0
    angle1 = 0:0.01:rad2deg(Euler(1));
else
    angle1 = 0:-0.01:rad2deg(Euler(1));
end
k = 1:length(angle1);
ko = (linspace(1,length(angle1),nframe))';
angle1b = interp1(k,angle1,ko,'makima');
clear k ko;
if Euler(2) > 0
    angle2 = 0:0.01:rad2deg(Euler(2));
else
    angle2 = 0:-0.01:rad2deg(Euler(2));
end
k = 1:length(angle2);
ko = (linspace(1,length(angle2),nframe))';
angle2b = interp1(k,angle2,ko,'makima');
clear k ko;
if Euler(3) > 0
    angle3 = 0:0.01:rad2deg(Euler(3));
else
    angle3 = 0:-0.01:rad2deg(Euler(3));
end
k       = 1:length(angle3);
ko      = (linspace(1,length(angle3),nframe))';
angle3b = interp1(k,angle3,ko,'makima');
clear k ko angle1 angle2 angle3;
for i = 1:length(angle1b)
    R1 = [cosd(angle1b(i)) -sind(angle1b(i)) 0; sind(angle1b(i)) cosd(angle1b(i)) 0; 0 0 1];
    R2 = [cosd(angle2b(i)) 0 sind(angle2b(i)); 0 1 0; -sind(angle2b(i)) 0 cosd(angle2b(i))];
    R3 = [1 0 0; 0 cosd(angle3b(i)) -sind(angle3b(i)); 0 sind(angle3b(i)) cosd(angle3b(i))];
    temp = T_ics_fs*[R1 [0 0 0]'; 0 0 0 1]*[R2 [0 0 0]'; 0 0 0 1]*[R3 [0 0 0]'; 0 0 0 1];
    Xf = [Xf; temp(1:3,1)'];
    Yf = [Yf; temp(1:3,2)'];
    Zf = [Zf; temp(1:3,3)'];
    if i/2 == round(i/2)
        signNoise = 1;
    else
        signNoise = -1;
    end
    Of = [Of; Ofs];%[Of; temp(1:3,4)'+signNoise*0.05]; % 0.05 mm noise to avoid point repetition
    clear R1 R2 R3;
end
clear angle1b angle2b angle3b temp;

% Set the initialised flange coordinate system fi
T_ics_fi = [Xf(end,:)' Yf(end,:)' Zf(end,:)' Of(end,:)'; 0 0 0 1];
T_fi_ris = inv(T_ics_fi)*T_ics_ris;
T_fi_ris(1:3,1:3) = [1 0 0; 0 1 0; 0 0 1]; % Remove potential numerical error on rotations

% % Set FE movement
% % -------------------------------------------------------------------------
% 
% % Set range of motion
% if contains(Subject.side,'R')
%     minAngle = -60; % Right side: Flexion (-60)
%     maxAngle = 60; % Right side: Extension (60)
% elseif contains(Subject.side,'L')
%     minAngle = -60; % Left side: Flexion (-60)
%     maxAngle = 60; % Left side: Extension (60)
% end
%     
% % Initialisation / 50 frames
% angle  = 0:-1:minAngle; % deg
% k      = 1:length(angle);
% ko     = (linspace(1,length(angle),50))';
% angleb = interp1(k,angle,ko,'makima');
% clear k ko angle;
% clear T_ics_f;
% for i = 2:length(angleb) % Start at 2 to avoid point repetition
%     T_ri_ris = [[cosd(angleb(i)) -sind(angleb(i)) 0; sind(angleb(i)) cosd(angleb(i)) 0; 0 0 1] [0 0 0]'; 0 0 0 1]; % rotation around Z_as axis
%     T_ics_f  = T_ics_ri*T_ri_ris*inv(T_fi_ris);
%     Xf       = [Xf; T_ics_f(1:3,1)'];
%     Yf       = [Yf; T_ics_f(1:3,2)'];
%     Zf       = [Zf; T_ics_f(1:3,3)'];
%     if i/2 == round(i/2)
%         signNoise = 1;
%     else
%         signNoise = -1;
%     end
%     Of = [Of; T_ics_f(1:3,4)'+signNoise*0.05];
%     clear T_ri_ris T_ics_f;
% end
% clear angleb;
% 
% % Range of motion / 100 frames per cycle
% nrepetition = 1;
% for irepetition = 1:nrepetition
%     angle  = [minAngle:1:maxAngle maxAngle:-1:minAngle]; % deg
%     k      = 1:length(angle);
%     ko     = (linspace(1,length(angle),100))';
%     angleb = interp1(k,angle,ko,'makima');
%     clear k ko angle;
%     clear T_ics_f;
%     for i = 2:length(angleb)
%     T_ri_ris = [[cosd(angleb(i)) -sind(angleb(i)) 0; sind(angleb(i)) cosd(angleb(i)) 0; 0 0 1] [0 0 0]'; 0 0 0 1]; % rotation around Z_as axis
%         T_ics_f  = T_ics_ri*T_ri_ris*inv(T_fi_ris);
%         Xf       = [Xf; T_ics_f(1:3,1)'];
%         Yf       = [Yf; T_ics_f(1:3,2)'];
%         Zf       = [Zf; T_ics_f(1:3,3)'];
%         if i/2 == round(i/2)
%             signNoise = 1;
%         else
%             signNoise = -1;
%         end
%         Of = [Of; T_ics_f(1:3,4)'+signNoise*0.05];
%         clear T_ri_ris T_ics_f;
%     end
%     clear angleb;
% end
% 
% % Back to initial orientation / 50 frames
% angle  = minAngle:1:0; % deg
% k      = 1:length(angle);
% ko     = (linspace(1,length(angle),50))';
% angleb = interp1(k,angle,ko,'makima');
% clear k ko angle;
% clear T_ics_f;
% for i = 2:length(angleb)
%     T_ri_ris = [[cosd(angleb(i)) -sind(angleb(i)) 0; sind(angleb(i)) cosd(angleb(i)) 0; 0 0 1] [0 0 0]'; 0 0 0 1]; % rotation around Z_as axis
%     T_ics_f  = T_ics_ri*T_ri_ris*inv(T_fi_ris);
%     Xf       = [Xf; T_ics_f(1:3,1)'];
%     Yf       = [Yf; T_ics_f(1:3,2)'];
%     Zf       = [Zf; T_ics_f(1:3,3)'];
%     if i/2 == round(i/2)
%         signNoise = 1;
%     else
%         signNoise = -1;
%     end
%     Of = [Of; T_ics_f(1:3,4)'+signNoise*0.05];
%     clear T_ri_ris T_ics_f;
% end
% clear angleb;
% 
% % Set PS movement
% % -------------------------------------------------------------------------
% 
% % Set range of motion
% if contains(Subject.side,'R')
%     minAngle = -60; % Right side: Supination (-60)
%     maxAngle = 60; % Right side: Pronation (60)
% elseif contains(Subject.side,'L')
%     minAngle = -60; % Left side: Pronation (-60)
%     maxAngle = 60; % Left side: Supination (60)
% end
% 
% % Initialisation / 50 frames
% angle  = 0:-1:minAngle; % deg
% k      = 1:length(angle);
% ko     = (linspace(1,length(angle),50))';
% angleb = interp1(k,angle,ko,'makima');
% clear k ko angle;
% clear T_ics_f;
% for i = 2:length(angleb)
%     T_ri_ris = [[cosd(angleb(i)) 0 sind(angleb(i)); 0 1 0; -sind(angleb(i)) 0 cosd(angleb(i))] [0 0 0]'; 0 0 0 1]; % rotation around Y_as axis
%     T_ics_f  = T_ics_ri*T_ri_ris*inv(T_fi_ris);
%     Xf       = [Xf; T_ics_f(1:3,1)'];
%     Yf       = [Yf; T_ics_f(1:3,2)'];
%     Zf       = [Zf; T_ics_f(1:3,3)'];
%     if i/2 == round(i/2)
%         signNoise = 1;
%     else
%         signNoise = -1;
%     end
%     Of = [Of; T_ics_f(1:3,4)'+signNoise*0.05];
%     clear T_ri_ris T_ics_f;
% end
% clear angleb;
% 
% % Range of motion / 100 frames per cycle
% nrepetition = 1;
% for irepetition = 1:nrepetition
%     angle  = [minAngle:1:maxAngle maxAngle:-1:minAngle]; % deg
%     k      = 1:length(angle);
%     ko     = (linspace(1,length(angle),100))';
%     angleb = interp1(k,angle,ko,'makima');
%     clear k ko angle;
%     clear T_ics_f;
%     for i = 2:length(angleb)
%         T_ri_ris = [[cosd(angleb(i)) 0 sind(angleb(i)); 0 1 0; -sind(angleb(i)) 0 cosd(angleb(i))] [0 0 0]'; 0 0 0 1]; % rotation around Y_as axis
%         T_ics_f  = T_ics_ri*T_ri_ris*inv(T_fi_ris);
%         Xf       = [Xf; T_ics_f(1:3,1)'];
%         Yf       = [Yf; T_ics_f(1:3,2)'];
%         Zf       = [Zf; T_ics_f(1:3,3)'];
%         if i/2 == round(i/2)
%             signNoise = 1;
%         else
%             signNoise = -1;
%         end
%         Of = [Of; T_ics_f(1:3,4)'+signNoise*0.05];
%         clear T_ri_ris T_ics_f;
%     end
%     clear angleb;
% end
% 
% % Back to initial orientation / 50 frames
% angle  = minAngle:1:0; % deg
% k      = 1:length(angle);
% ko     = (linspace(1,length(angle),50))';
% angleb = interp1(k,angle,ko,'makima');
% clear k ko angle;
% clear T_ics_f;
% for i = 2:length(angleb)
%     T_ri_ris = [[cosd(angleb(i)) 0 sind(angleb(i)); 0 1 0; -sind(angleb(i)) 0 cosd(angleb(i))] [0 0 0]'; 0 0 0 1]; % rotation around Y_as axis
%     T_ics_f  = T_ics_ri*T_ri_ris*inv(T_fi_ris);
%     Xf       = [Xf; T_ics_f(1:3,1)'];
%     Yf       = [Yf; T_ics_f(1:3,2)'];
%     Zf       = [Zf; T_ics_f(1:3,3)'];
%     if i/2 == round(i/2)
%         signNoise = 1;
%     else
%         signNoise = -1;
%     end
%     Of = [Of; T_ics_f(1:3,4)'+signNoise*0.05];
%     clear T_ri_ris T_ics_f;
% end
% clear angleb;

% Set RUD movement
% -------------------------------------------------------------------------

% Set range of motion
if contains(Subject.side,'R')
    minAngle = -30; % Right side: Radius deviation (-15)
    maxAngle = 15; % Right side: Ulna deviation (30)
elseif contains(Subject.side,'L')
    minAngle = -15; % Left side: Radius deviation (-30)
    maxAngle = 30; % Left side: Ulna deviation (15)
end

% Initialisation / 50 frames
angle  = 0:-1:minAngle; % deg
k      = 1:length(angle);
ko     = (linspace(1,length(angle),50))';
angleb = interp1(k,angle,ko,'makima');
clear k ko angle;
clear T_ics_f;
for i = 2:length(angleb)
    T_ri_ris = [[1 0 0; 0 cosd(angleb(i)) -sind(angleb(i)); 0 sind(angleb(i)) cosd(angleb(i))] [0 0 0]'; 0 0 0 1]; % rotation around X_as axis
    T_ics_f  = T_ics_ri*T_ri_ris*inv(T_fi_ris);
    Xf       = [Xf; T_ics_f(1:3,1)'];
    Yf       = [Yf; T_ics_f(1:3,2)'];
    Zf       = [Zf; T_ics_f(1:3,3)'];
    if i/2 == round(i/2)
        signNoise = 1;
    else
        signNoise = -1;
    end
    Of = [Of; T_ics_f(1:3,4)'+signNoise*0.05];
    clear T_ri_ris T_ics_f;
end
clear angleb;

% Range of motion / 100 frames per cycle
nrepetition = 1;
for irepetition = 1:nrepetition
    angle  = [minAngle:1:maxAngle maxAngle:-1:minAngle]; % deg
    k      = 1:length(angle);
    ko     = (linspace(1,length(angle),100))';
    angleb = interp1(k,angle,ko,'makima');
    clear k ko angle;
    clear T_ics_f;
    for i = 2:length(angleb)
        T_ri_ris = [[1 0 0; 0 cosd(angleb(i)) -sind(angleb(i)); 0 sind(angleb(i)) cosd(angleb(i))] [0 0 0]'; 0 0 0 1]; % rotation around X_as axis
        T_ics_f  = T_ics_ri*T_ri_ris*inv(T_fi_ris);
        Xf       = [Xf; T_ics_f(1:3,1)'];
        Yf       = [Yf; T_ics_f(1:3,2)'];
        Zf       = [Zf; T_ics_f(1:3,3)'];
        if i/2 == round(i/2)
            signNoise = 1;
        else
            signNoise = -1;
        end
        Of = [Of; T_ics_f(1:3,4)'+signNoise*0.05];
        clear T_ri_ris T_ics_f;
    end
    clear angleb;
end

% Back to initial orientation / 50 frames
angle  = minAngle:1:0; % deg
k      = 1:length(angle);
ko     = (linspace(1,length(angle),50))';
angleb = interp1(k,angle,ko,'makima');
clear k ko angle;
clear T_ics_f;
for i = 2:length(angleb)
    T_ri_ris = [[1 0 0; 0 cosd(angleb(i)) -sind(angleb(i)); 0 sind(angleb(i)) cosd(angleb(i))] [0 0 0]'; 0 0 0 1]; % rotation around X_as axis
    T_ics_f  = T_ics_ri*T_ri_ris*inv(T_fi_ris);
    Xf       = [Xf; T_ics_f(1:3,1)'];
    Yf       = [Yf; T_ics_f(1:3,2)'];
    Zf       = [Zf; T_ics_f(1:3,3)'];
    if i/2 == round(i/2)
        signNoise = 1;
    else
        signNoise = -1;
    end
    Of = [Of; T_ics_f(1:3,4)'+signNoise*0.05];
    clear T_ri_ris T_ics_f;
end
clear angleb;

% -------------------------------------------------------------------------
% STORE DATA FOR THE ROBOT INVERSE KINEMATICS PROCESS
% ------------------------------------------------------------------------- 

% Set movement related timing
% ------------------------------------------------------------------------- 
iframe = 1:size(Xf,1);      

% Data in meter
% ------------------------------------------------------------------------- 
for i = 1:length(markerNames)
    Marker.(markerNames{i}) = Marker.(markerNames{i})*1e-3; % Markers coordinates (m)
end
Of = Of*1e-3;

% Permute axes to fit the robot coordinate systems definition
% ------------------------------------------------------------------------- 
t1 = Xf;
t2 = Yf;
t3 = Zf;
Xf = -t3;
Yf = -t1;
Zf = t2;

% Store last axis position and orientation     
% -------------------------------------------------------------------------             
RA6  = Of - Zf.*(Robot.lenghtFlange*1e-3); 
radiusRobotAxis6   = sqrt(RA6(:,1).^2 + ...
                          RA6(:,2).^2 + ...
                          (RA6(:,3)-Robot.heightWorkspaceCentre*1e-3).^2);
                      
% % Plot robot axis 6 radius
% % ------------------------------------------------------------------------- 
% figure;
% hold on; grid on;
% title('Distance from iiwa workspace centre of robot axis 6 (mm)');
% xlabel('Frames');
% plot(radiusRobotAxis6*1e3);
% line([0 nMarker],[Robot.minRadius Robot.minRadius],'Color','red','LineStyle','--');
% line([0 nMarker],[Robot.maxRadius Robot.maxRadius],'Color','red','LineStyle','--');
% legend('Robot axis 6 radius','Lower limit','Upper limit');
%      
% % Set 3D figure
% % ------------------------------------------------------------------------- 
% figure;
% hold on; grid on; axis equal;
% title('Final position of the specimen positioning problem');  
% 
% % Plot humerus support trajectories during frames of interest
% % -------------------------------------------------------------------------
% Of = Of*1e3;
% RA6 = RA6*1e3;
% plot3(Of(iframe,1),Of(iframe,2),Of(iframe,3),'+','LineWidth',2);
% plot3(RA6(iframe,1),RA6(iframe,2),RA6(iframe,3),'+','LineWidth',2);
% plot3(RA6(:,1),RA6(:,2),RA6(:,3),'Color','black','LineStyle','--','LineWidth',1);
% 
% % Plot robot geometrical workspace
% % -------------------------------------------------------------------------
% plot3(sqrt(Robot.maxRadius^2-([-600:10:600].^2)),[-600:10:600],Robot.heightWorkspaceCentre*ones(1,121),...
%      'Color','cyan','LineStyle','--','LineWidth',1);
% plot3(sqrt(Robot.minRadius^2-([-400:10:400].^2)),[-400:10:400],Robot.heightWorkspaceCentre*ones(1,81),...
%       'Color','cyan','LineStyle','--','LineWidth',1);
% plot3(sqrt(Robot.maxRadius^2-([-600:10:600].^2)),zeros(1,121),[-600:10:600]+Robot.heightWorkspaceCentre,...
%      'Color','cyan','LineStyle','--','LineWidth',1);
% plot3(sqrt(Robot.minRadius^2-([-400:10:400].^2)),zeros(1,81),[-400:10:400]+Robot.heightWorkspaceCentre,...
%       'Color','cyan','LineStyle','--','LineWidth',1); 
% clear iframe;
% 
% % Plot robot coordinate system 
% % -------------------------------------------------------------------------  
% quiver3(0,0,0,1,0,0,200,'Color','red');
% quiver3(0,0,0,0,1,0,200,'Color','green');
% quiver3(0,0,0,0,0,1,200,'Color','blue');

% Store flange position and orientation for the current movement 
% ------------------------------------------------------------------------- 
O_flange        = permute(Of,[2,3,1]);
R_flange        = [permute(Xf,[2,3,1]) permute(Yf,[2,3,1]) permute(Zf,[2,3,1])];
temp            = permute(rotMat2quatern(R_flange),[2,3,1]);
Q_flange(1,1,:) = temp(2,:,:);
Q_flange(2,1,:) = temp(3,:,:);
Q_flange(3,1,:) = temp(4,:,:);
Q_flange(4,1,:) = temp(1,:,:);
% Continuity test (Lee's method)
% Quaternions q and -q can represent the same rotation R
for i = 2:size(Q_flange,3)
    % ||log(qi-1)-1*qi)|| > pi/2
    if norm(qlog_array3(qprod_array3(...
        qinv_array3(Q_flange(:,:,i-1)), Q_flange(:,:,i)))) > pi/2
        % Quaternion of opposite sign
        Q_flange(:,:,i) = - Q_flange(:,:,i);
    end
end
Q_flange(4,1,:) = -Q_flange(4,1,:);
% Normalise the quaternion
for i = 1:size(Q_flange,3)
    Q_flange(:,:,i) = Q_flange(:,:,i)./sqrt(Q_flange(1,1,i)^2+...
                                            Q_flange(2,1,i)^2+...
                                            Q_flange(3,1,i)^2+...
                                            Q_flange(4,1,i)^2);
end
clear temp;

% Export results 
% ------------------------------------------------------------------------- 
cd(Folder.export);
Mvt       = 'ALL';
Ord       = '02';   
Name      = [Ord,'_',Mvt,'_',Subject.side,'.mat'];
Namefull  = [Mvt,'_',Subject.side,'_','full.mat'];
save(Name,'O_flange','Q_flange');
save(Namefull);
clear Name Namefull Num Side Ord Mvt;

%% ------------------------------------------------------------------------
% PLOT 3D
% ------------------------------------------------------------------------- 
figure;
founder = 0;
for i = iframe(1):10:iframe(end)
    plot3(Marker.END01(:,1),Marker.END01(:,2),Marker.END01(:,3),'Marker','o','Color','red');
    hold on;
    plot3(Marker.END02(:,1),Marker.END02(:,2),Marker.END02(:,3),'Marker','o','Color','red');
    plot3(Marker.END03(:,1),Marker.END03(:,2),Marker.END03(:,3),'Marker','o','Color','red');
    plot3(Marker.END04(:,1),Marker.END04(:,2),Marker.END04(:,3),'Marker','o','Color','red');
    plot3(Marker.BAS01(:,1),Marker.BAS01(:,2),Marker.BAS01(:,3),'Marker','o','Color','green');
    plot3(Marker.BAS02(:,1),Marker.BAS02(:,2),Marker.BAS02(:,3),'Marker','o','Color','green');
    plot3(Marker.BAS03(:,1),Marker.BAS03(:,2),Marker.BAS03(:,3),'Marker','o','Color','green');
    plot3(Marker.BAS04(:,1),Marker.BAS04(:,2),Marker.BAS04(:,3),'Marker','o','Color','green');
    plot3(Marker.BAS05(:,1),Marker.BAS05(:,2),Marker.BAS05(:,3),'Marker','o','Color','green');
    plot3(Marker.EM(:,1),Marker.EM(:,2),Marker.EM(:,3),'Marker','o','Color','blue');
    plot3(Marker.EL(:,1),Marker.EL(:,2),Marker.EL(:,3),'Marker','o','Color','blue');
    plot3(Marker.US(:,1),Marker.US(:,2),Marker.US(:,3),'Marker','o','Color','blue');
    plot3(Marker.RS(:,1),Marker.RS(:,2),Marker.RS(:,3),'Marker','o','Color','blue');
    plot3(RA6(i,1),RA6(i,2),RA6(i,3),'Marker','o','Color','magenta');
    plot3(Of(i,1),Of(i,2),Of(i,3),'Marker','o','Color','black');
%     quiver3(Ofs(1,1)*1e-3,Ofs(1,2)*1e-3,Ofs(1,3)*1e-3,...
%             Xfs(1,1),Xfs(1,2),Xfs(1,3),...
%             0.2,'Color','red');
%     quiver3(Ofs(1,1)*1e-3,Ofs(1,2)*1e-3,Ofs(1,3)*1e-3,...
%             Yfs(1,1),Yfs(1,2),Yfs(1,3),...
%             0.2,'Color','green');
%     quiver3(Ofs(1,1)*1e-3,Ofs(1,2)*1e-3,Ofs(1,3)*1e-3,...
%             Zfs(1,1),Zfs(1,2),Zfs(1,3),...
%             0.2,'Color','blue'); 
    quiver3(Oris(1,1)*1e-3,Oris(1,2)*1e-3,Oris(1,3)*1e-3,...
            Xris(1,1),Xris(1,2),Xris(1,3),...
            0.2,'Color','red');
    quiver3(Oris(1,1)*1e-3,Oris(1,2)*1e-3,Oris(1,3)*1e-3,...
            Yris(1,1),Yris(1,2),Yris(1,3),...
            0.2,'Color','green');
    quiver3(Oris(1,1)*1e-3,Oris(1,2)*1e-3,Oris(1,3)*1e-3,...
            Zris(1,1),Zris(1,2),Zris(1,3),...
            0.2,'Color','blue');
    quiver3(Of(i,1),Of(i,2),Of(i,3),...
            Xf(i,1),Xf(i,2),Xf(i,3),...
            0.2,'Color','red','Linestyle','--');
    quiver3(Of(i,1),Of(i,2),Of(i,3),...
            Yf(i,1),Yf(i,2),Yf(i,3),...
            0.2,'Color','green','Linestyle','--');
    quiver3(Of(i,1),Of(i,2),Of(i,3),...
            Zf(i,1),Zf(i,2),Zf(i,3),...
            0.2,'Color','blue','Linestyle','--');   
    axis equal;
    grid on;
    box on;
    xlabel('X axis');
    ylabel('Y axis');
    zlabel('Z axis');
%     xlim([0.2 0.8]);
%     ylim([-0.3 0.3]);
%     zlim([0 0.9]);
    view(30,0); 
    pause(0.1);
    hold off;
end