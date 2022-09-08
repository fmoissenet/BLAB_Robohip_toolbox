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
Folder.data    = 'C:\Users\moissene\OneDrive - unige.ch\2022 - ROBOHIP\Données\Test20220907\plastic\';
Folder.export  = 'C:\Users\moissene\OneDrive - unige.ch\2022 - ROBOHIP\Données\Test20220907\plastic\';
Folder.dep     = [Folder.toolbox,'dependencies\'];
addpath(Folder.toolbox);
addpath(genpath(Folder.dep));
cd(Folder.data);

% -------------------------------------------------------------------------
% SET SUBJECT INFO
% -------------------------------------------------------------------------
Subject.id   = 'RH000';
Subject.side = 'L'; % R or L

% -------------------------------------------------------------------------
% SET ROBOT PARAMETERS
% -------------------------------------------------------------------------
Robot.ThicknessBotaPlate = 10; % Thickness of the Bota sensor plate (mm)

% -------------------------------------------------------------------------
% DEFINE THE RIGID TRANSFORMATION BETWEEN MOCAP ICS AND ROBOT BASE
% -------------------------------------------------------------------------
% Load marker trajectories stored in C3D files
% -------------------------------------------------------------------------
[Marker,Event,firstFrame,nMarker,fMarker,markerNames] = LoadC3DFile('robotCalibration01.c3d');

% Compute mean position across all frames
% -------------------------------------------------------------------------
for imarker = 1:size(markerNames,1)
    Marker.(markerNames{imarker}) = mean(Marker.(markerNames{imarker}),3);
end

% Set virtual marker positions based on geometric assumptions
% -------------------------------------------------------------------------
dMarker       = 14;                % Diameter of the robot base plate markers (mm)
Xbase         = [1 0 0]';
Ybase         = [0 1 0]';
Zbase         = [0 0 1]';
Obase         = [0.0 0.0 0.0]';    % Based on technical documentation (mm)
Vmarker.BAS01 = Obase ...
                + 340/2*Xbase ...  % Defined on CAO documentation fromm HEPIA (mm)
                - 220/2*Ybase ...  % Defined on CAO documentation fromm HEPIA (mm)
                - 1*Zbase;         % Depth of the marker base placement on the flange plate (mm)
                + dMarker/2*Zbase;
Vmarker.BAS02 = Obase ...
                - 340/2*Xbase ...  % Defined on CAO documentation fromm HEPIA (mm)
                - 220/2*Ybase ...  % Defined on CAO documentation fromm HEPIA (mm)
                - 1*Zbase;         % Depth of the marker base placement on the flange plate (mm)
                + dMarker/2*Zbase;
Vmarker.BAS03 = Obase ...
                - 340/2*Xbase ...  % Defined on CAO documentation fromm HEPIA (mm)
                + 220/2*Ybase ...  % Defined on CAO documentation fromm HEPIA (mm)
                - 1*Zbase;         % Depth of the marker base placement on the flange plate (mm)
                + dMarker/2*Zbase;
Vmarker.BAS04 = Obase ...
                + 340/2*Xbase ...  % Defined on CAO documentation fromm HEPIA (mm)
                + 220/2*Ybase ...  % Defined on CAO documentation fromm HEPIA (mm)
                - 1*Zbase;         % Depth of the marker base placement on the flange plate (mm)
                + dMarker/2*Zbase;

% Compute the rigid transformation between coordinate systems
% -------------------------------------------------------------------------
[R_qualisys_robot,d_qualisys_robot,rms] = soder([Marker.BAS01';Marker.BAS02';Marker.BAS03';Marker.BAS04'], ...
                                                [Vmarker.BAS01';Vmarker.BAS02';Vmarker.BAS03';Vmarker.BAS04']);
T_qualisys_robot = [R_qualisys_robot d_qualisys_robot; 0 0 0 1];

% Clear workspace
% -------------------------------------------------------------------------
clearvars -except Folder Subject Robot T_qualisys_robot;

% -------------------------------------------------------------------------
% SET THE TIP LOCATION OF THE STYLUS
% -------------------------------------------------------------------------

% Load marker trajectories stored in C3D files
% -------------------------------------------------------------------------
[Marker,Event,firstFrame,nMarker,fMarker,markerNames] = LoadC3DFile('stylusCalibration01.c3d');

% Compute mean position across all frames (except Stylus markers)
% -------------------------------------------------------------------------
for imarker = 1:size(markerNames,1)
    if isempty(strfind(markerNames{imarker},'STY'))
        Marker.(markerNames{imarker}) = mean(Marker.(markerNames{imarker}),1);
    end
end

% Define stylus coordinate system
% -------------------------------------------------------------------------
Os      = Marker.STY04;
Ys      = Vnorm_array3(Marker.STY02-Marker.STY05);
Xs      = Vnorm_array3(cross(Marker.STY03-Marker.STY05,Marker.STY01-Marker.STY05));
Zs      = Vnorm_array3(cross(Xs,Ys));
T_ics_s = [Xs Ys Zs Os; zeros(1,3,nMarker) ones(1,1,nMarker)]; % from s to ics

% Compute the averaged centre of rotation between T_ics_s and T_ics_ics
% -------------------------------------------------------------------------
T_ics_ics               = repmat([[1 0 0; 0 1 0; 0 0 1] [0 0 0]'; 0 0 0 1],[1 1 nMarker]); % from ics to ics
[rC,rCsi,rCsj,Residual] = SCoRE_array3(T_ics_s,T_ics_ics);
Stylus.STY06            = rCsi; % Store local position

% Clear workspace
% -------------------------------------------------------------------------
clearvars -except Folder Subject Robot T_qualisys_robot Stylus;

% -------------------------------------------------------------------------
% DEFINE VIRTUAL MARKER AT BONY LANDMARKS
% -------------------------------------------------------------------------

% Load marker trajectories stored in C3D files
% -------------------------------------------------------------------------
[Marker,Event,firstFrame,nMarker,fMarker,markerNames] = LoadC3DFile('boneCalibration01.c3d');

% Compute mean position across all frames (except Stylus markers)
% -------------------------------------------------------------------------
for imarker = 1:size(markerNames,1)
    if isempty(strfind(markerNames{imarker},'STY'))
        Marker.(markerNames{imarker}) = mean(Marker.(markerNames{imarker}),3);
    end
end

% Define stylus coordinate system
% -------------------------------------------------------------------------
Os      = Marker.STY04;
Ys      = Vnorm_array3(Marker.STY02-Marker.STY05);
Xs      = Vnorm_array3(cross(Marker.STY03-Marker.STY05,Marker.STY01-Marker.STY05));
Zs      = Vnorm_array3(cross(Xs,Ys));
T_ics_s = [Xs Ys Zs Os; zeros(1,3,nMarker) ones(1,1,nMarker)]; % from s to ics

% Compute the virtual marker corresponding to the stylus tip
% -------------------------------------------------------------------------
temp         = Mprod_array3(T_ics_s,repmat([Stylus.STY06; 1],[1 1 nMarker]));
Marker.STY06 = temp(1:3,:,:);
clear temp;

% Get the global position of the tip at key events
% -------------------------------------------------------------------------
landmarks = {'RIAS' 'LIAS' 'RIPS' 'LIPS' 'FLE' 'FME'};
for ilandmark = 1:size(landmarks,2)
    Marker.(landmarks{ilandmark}) = Marker.STY06(:,:,round(Event.(landmarks{ilandmark})*fMarker)-firstFrame+1);
end
markerNames = fieldnames(Marker);

% Extract landmarks and cluster markers
% -------------------------------------------------------------------------
Calibration.Pelvis.cluster   = [Marker.cPEL01 Marker.cPEL02 Marker.cPEL03 Marker.cPEL04];
Calibration.Pelvis.landmarks = [Marker.RIAS Marker.LIAS Marker.RIPS Marker.LIPS];
Calibration.Femur.cluster    = [Marker.cFEM01 Marker.cFEM02 Marker.cFEM03 Marker.cFEM04];
Calibration.Femur.landmarks  = [Marker.FLE Marker.FME];

% Export from camera coordinate system to robot coordinate system
% -------------------------------------------------------------------------
Calibration.Pelvis.cluster   = Mprod_array3(Minv_array3(T_qualisys_robot),[Calibration.Pelvis.cluster; ones(1,size(Calibration.Pelvis.cluster,2),1)]);
Calibration.Pelvis.cluster   = Calibration.Pelvis.cluster(1:3,:,:);
Calibration.Pelvis.landmarks = Mprod_array3(Minv_array3(T_qualisys_robot),[Calibration.Pelvis.landmarks; ones(1,size(Calibration.Pelvis.landmarks,2),1)]);
Calibration.Pelvis.landmarks = Calibration.Pelvis.landmarks(1:3,:,:);
Calibration.Femur.cluster    = Mprod_array3(Minv_array3(T_qualisys_robot),[Calibration.Femur.cluster; ones(1,size(Calibration.Femur.cluster,2),1)]);
Calibration.Femur.cluster    = Calibration.Femur.cluster(1:3,:,:);
Calibration.Femur.landmarks  = Mprod_array3(Minv_array3(T_qualisys_robot),[Calibration.Femur.landmarks; ones(1,size(Calibration.Femur.landmarks,2),1)]);
Calibration.Femur.landmarks  = Calibration.Femur.landmarks(1:3,:,:);

% Clear workspace
% -------------------------------------------------------------------------
clearvars -except Folder Subject Robot T_qualisys_robot Stylus Calibration;

% -------------------------------------------------------------------------
% DEFINE HIP JOINT CENTRE
% -------------------------------------------------------------------------

% Load marker trajectories stored in C3D files
% -------------------------------------------------------------------------
[Marker,Event,firstFrame,nMarker,fMarker,markerNames] = LoadC3DFile('boneFunctional01.c3d');

% Extract cluster markers
% -------------------------------------------------------------------------
Functional.Pelvis.cluster = [Marker.cPEL01 Marker.cPEL02 Marker.cPEL03 Marker.cPEL04];
Functional.Femur.cluster  = [Marker.cFEM01 Marker.cFEM02 Marker.cFEM03 Marker.cFEM04];

% Compute landmarks
% -------------------------------------------------------------------------
for t = 1:nMarker
    [R,d,rms] = soder(Calibration.Pelvis.cluster',Functional.Pelvis.cluster(:,:,t)');
    Functional.Pelvis.landmarks(:,:,t) = R*Calibration.Pelvis.landmarks+d;
    clear R d rms;
    [R(:,:,t),d(:,:,t),rms] = soder(Calibration.Femur.cluster',Functional.Femur.cluster(:,:,t)');
    Functional.Femur.landmarks(:,:,t) = R(:,:,t)*Calibration.Femur.landmarks+d(:,:,t);
    clear R d rms;
end

% Compute instantaneous centre of rotations
% -------------------------------------------------------------------------
% Set pelvis technical coordinate system
Xp = Vnorm_array3(Functional.Pelvis.cluster(:,2,:)-Functional.Pelvis.cluster(:,1,:));
Zp = Vnorm_array3(cross(Functional.Pelvis.cluster(:,3,:)-Functional.Pelvis.cluster(:,4,:), ...
                        Functional.Pelvis.cluster(:,1,:)-Functional.Pelvis.cluster(:,4,:)));
Yp = Vnorm_array3(cross(Zp,Xp));
Rp = [Xp Yp Zp];
Op = Functional.Pelvis.cluster(:,1,:);
Tp = [Rp Op; repmat([0 0 0 1],[1,1,nMarker])];
% Set femur technical coordinate system
Xf = Vnorm_array3(Functional.Femur.cluster(:,2,:)-Functional.Femur.cluster(:,1,:));
Zf = Vnorm_array3(cross(Functional.Femur.cluster(:,3,:)-Functional.Femur.cluster(:,4,:), ...
                         Functional.Femur.cluster(:,1,:)-Functional.Femur.cluster(:,4,:)));
Yf = Vnorm_array3(cross(Zf,Xf));
Rf = [Xf Yf Zf];
Of = Functional.Femur.cluster(:,1,:);
Tf = [Rf Of; repmat([0 0 0 1],[1,1,nMarker])];
% Compute instantaneous centre of rotations
[rC,rCsi,rCsj,Residual]    = SCoRE_array3(Tf,Tp);
Functional.Femur.landmarks = [Functional.Femur.landmarks rC];
clear rC rCsi rCsj Residual;

% Export from camera coordinate system to robot coordinate system
% -------------------------------------------------------------------------
Functional.Pelvis.cluster   = Mprod_array3(Minv_array3(repmat(T_qualisys_robot,[1 1 nMarker])),[Functional.Pelvis.cluster; ones(1,size(Functional.Pelvis.cluster,2),nMarker)]);
Functional.Pelvis.cluster   = Functional.Pelvis.cluster(1:3,:,:);
Functional.Pelvis.landmarks = Mprod_array3(Minv_array3(repmat(T_qualisys_robot,[1 1 nMarker])),[Functional.Pelvis.landmarks; ones(1,size(Functional.Pelvis.landmarks,2),nMarker)]);
Functional.Pelvis.landmarks = Functional.Pelvis.landmarks(1:3,:,:);
Functional.Femur.cluster    = Mprod_array3(Minv_array3(repmat(T_qualisys_robot,[1 1 nMarker])),[Functional.Femur.cluster; ones(1,size(Functional.Femur.cluster,2),nMarker)]);
Functional.Femur.cluster    = Functional.Femur.cluster(1:3,:,:);
Functional.Femur.landmarks  = Mprod_array3(Minv_array3(repmat(T_qualisys_robot,[1 1 nMarker])),[Functional.Femur.landmarks; ones(1,size(Functional.Femur.landmarks,2),nMarker)]);
Functional.Femur.landmarks  = Functional.Femur.landmarks(1:3,:,:);

% Clear workspace
% -------------------------------------------------------------------------
clearvars -except Folder Subject Robot T_qualisys_robot Stylus Calibration Functional;

% -------------------------------------------------------------------------
% SET INITIAL POSITION
% -------------------------------------------------------------------------

% Load marker trajectories stored in C3D files
% -------------------------------------------------------------------------
[Marker,Event,firstFrame,nMarker,fMarker,markerNames] = LoadC3DFile('boneStatic01.c3d');

% Compute mean position across all frames
% -------------------------------------------------------------------------
for imarker = 1:size(markerNames,1)
    Marker.(markerNames{imarker}) = mean(Marker.(markerNames{imarker}),3);
end

% Extract cluster markers
% -------------------------------------------------------------------------
Static.Pelvis.cluster = [Marker.cPEL01 Marker.cPEL02 Marker.cPEL03 Marker.cPEL04];
Static.Femur.cluster  = [Marker.cFEM01 Marker.cFEM02 Marker.cFEM03 Marker.cFEM04];

% Compute landmarks in ICS
% -------------------------------------------------------------------------
[R,d,rms] = soder(Functional.Pelvis.cluster(:,:,1)',Static.Pelvis.cluster');
Static.Pelvis.landmarks = R*Functional.Pelvis.landmarks(:,:,1)+d;
clear R d rms;
[R,d,rms] = soder(Functional.Femur.cluster(:,:,1)',Static.Femur.cluster');
Static.Femur.landmarks = R*Functional.Femur.landmarks(:,:,1)+d;
clear R d rms;

% Set END markers barycentre
% -------------------------------------------------------------------------
Marker.END05 = (Marker.END01+Marker.END02+Marker.END03+Marker.END04)./4;

% Define robot flange markers (also considered as femur landmarks)
% -------------------------------------------------------------------------
Static.Femur.landmarks = [Static.Femur.landmarks ...
                          Marker.END01 Marker.END02 Marker.END03 Marker.END04 Marker.END05];

% Export from camera coordinate system to robot coordinate system
% -------------------------------------------------------------------------
Static.Pelvis.cluster   = Mprod_array3(Minv_array3(T_qualisys_robot),[Static.Pelvis.cluster; ones(1,size(Static.Pelvis.cluster,2),1)]);
Static.Pelvis.cluster   = Static.Pelvis.cluster(1:3,:,:);
Static.Pelvis.landmarks = Mprod_array3(Minv_array3(T_qualisys_robot),[Static.Pelvis.landmarks; ones(1,size(Static.Pelvis.landmarks,2),1)]);
Static.Pelvis.landmarks = Static.Pelvis.landmarks(1:3,:,:);
Static.Femur.cluster    = Mprod_array3(Minv_array3(T_qualisys_robot),[Static.Femur.cluster; ones(1,size(Static.Femur.cluster,2),1)]);
Static.Femur.cluster    = Static.Femur.cluster(1:3,:,:);
Static.Femur.landmarks  = Mprod_array3(Minv_array3(T_qualisys_robot),[Static.Femur.landmarks; ones(1,size(Static.Femur.landmarks,2),1)]);
Static.Femur.landmarks  = Static.Femur.landmarks(1:3,:,:);

% -------------------------------------------------------------------------
% DEFINE ANATOMICAL POSTURE
% -------------------------------------------------------------------------

% Define hip joint coordinate system (JCS) at initial posture (ref)
% -------------------------------------------------------------------------
e1           = Vnorm_array3(Static.Pelvis.landmarks(:,1)-Static.Pelvis.landmarks(:,2)); % Flexion/extension axis
e3           = Vnorm_array3(Static.Femur.landmarks(:,3)-(Static.Femur.landmarks(:,1)+Static.Femur.landmarks(:,2))/2); % Internal/external axis
e2           = Vnorm_array3(cross(e3,e1)); % Abduction/adduction axis
T_ics_hipref = [e1 e2 e3 Static.Femur.landmarks(:,3); 0 0 0 1];

% Compute femur landmarks in hip JCS
% -------------------------------------------------------------------------
temp = Mprod_array3(Minv_array3(T_ics_hipref),[Static.Femur.landmarks; ones(1,size(Static.Femur.landmarks,2),1)]);
Static.Femur.localLandmarks = temp(1:3,:,:);
clear temp;

% Define pelvis coordinate system
% -------------------------------------------------------------------------
Zp      = Vnorm_array3(Static.Pelvis.landmarks(:,1)-Static.Pelvis.landmarks(:,2));
Yp      = Vnorm_array3(cross(Static.Pelvis.landmarks(:,1)-Static.Pelvis.landmarks(:,3),Static.Pelvis.landmarks(:,2)-Static.Pelvis.landmarks(:,3)));
Xp      = Vnorm_array3(cross(Yp,Zp));
Rp      = [Xp Yp Zp];
Op      = (Static.Pelvis.landmarks(:,1)+Static.Pelvis.landmarks(:,2))/2;
T_ics_p = [Rp Op; 0 0 0 1];

% Define femur coordinate system
% -------------------------------------------------------------------------
Yf      = Vnorm_array3(Static.Femur.landmarks(:,3)-(Static.Femur.landmarks(:,1)+Static.Femur.landmarks(:,2))/2);
Xf      = Vnorm_array3(cross(Static.Femur.landmarks(:,2)-Static.Femur.landmarks(:,3),Static.Femur.landmarks(:,1)-Static.Femur.landmarks(:,3)));
Zf      = Vnorm_array3(cross(Xf,Yf));
Rf      = [Xf Yf Zf];
Of      = Static.Femur.landmarks(:,3);
T_ics_f = [Rf Of; 0 0 0 1];

% Compute angle difference (minus Euler angles of current posture)
% -------------------------------------------------------------------------
Thip = Mprod_array3(Tinv_array3(T_ics_p),T_ics_f);
Euler = -rad2deg(R2mobileZXY_array3(Thip(1:3,1:3,:)));

% Clear workspace
% -------------------------------------------------------------------------
clearvars -except Folder Subject Robot T_qualisys_robot Stylus Calibration Functional Static Euler T_ics_hipref;

% -------------------------------------------------------------------------
% MOTION SET 1
% Move to anatomical posture
% -------------------------------------------------------------------------
% Initialisation
% -------------------------------------------------------------------------
Flange = []; % Special case for motion set 1
Mvt    = 'MotionSet01';
Ord    = '01';

% Set consecutive motions to reach anatomical posture
% -------------------------------------------------------------------------
% Store motion first frame
start = 2; % Special case for motion set 1
% Precision
precision = 0.1; % deg
% Add a motion /Z (e1)
angles    = 0:sign(Euler(1))*precision:Euler(1); % (deg)
eulerAxis = 'e1';
[Static,T_ics_hipref] = AddMotion(angles,eulerAxis,Static,T_ics_hipref);
clear angles eulerAxis;
% Add a motion /X (e2)
angles    = 0:sign(Euler(2))*precision:Euler(2); % (deg)
eulerAxis = 'e2';
[Static,T_ics_hipref] = AddMotion(angles,eulerAxis,Static,T_ics_hipref);
clear angles eulerAxis;
% Add a motion /Y (e3)
angles    = 0:sign(Euler(3))*precision:Euler(3); % (deg)
eulerAxis = 'e3';
[Static,T_ics_hipref] = AddMotion(angles,eulerAxis,Static,T_ics_hipref);
% Store motion last frame
stop = size(Static.Femur.landmarks,3);
% Clear workspace
clear angles eulerAxis;

% Store flange position and orientation for the current movement (in m)
% -------------------------------------------------------------------------
Flange = ComputeFlangePosition(Flange,Static,Subject,Robot,Folder,Ord,Mvt,start,stop);

% Clear workspace
% -------------------------------------------------------------------------
clearvars -except Folder Subject Robot T_qualisys_robot Stylus Calibration Functional Static Flange T_ics_hipref start stop;

% -------------------------------------------------------------------------
% MOTION SET 2
% Induce hip flexion (30°)
% -------------------------------------------------------------------------
% Initialisation
% -------------------------------------------------------------------------
Mvt = 'MotionSet02';
Ord = '02';  

% Set consecutive motions to reach anatomical posture
% -------------------------------------------------------------------------
% Store motion first frame
start = stop+1; % Special case for motion set 1
% Precision
precision = 0.1; % deg
% Add a motion: flexion around hip JCS flexion/extension axis
angles    = 0:precision:30; % (deg)
eulerAxis = 'e1';
[Static,T_ics_hipref] = AddMotion(angles,eulerAxis,Static,T_ics_hipref);
% Store motion last frame
stop = size(Static.Femur.landmarks,3);
% Clear workspace
clear angles eulerAxis;

% Store flange position and orientation for the current movement (in m)
% -------------------------------------------------------------------------
Flange = ComputeFlangePosition(Flange,Static,Subject,Robot,Folder,Ord,Mvt,start,stop);

% Clear workspace
% -------------------------------------------------------------------------
clearvars -except Folder Subject Robot T_qualisys_robot Stylus Calibration Functional Static Flange T_ics_hipref start stop;

% -------------------------------------------------------------------------
% PLOT (ONLY FOR TEST)
% Plot all consecutive motion sets
% -------------------------------------------------------------------------
close all;
figure;
for t = 1:size(Static.Femur.landmarks,3)
    % Plot pelvis
    plot3(Static.Pelvis.cluster(1,:,1),...
          Static.Pelvis.cluster(2,:,1),...
          Static.Pelvis.cluster(3,:,1),...
          'Marker','.','Markersize',15,'Linestyle','none','Color','red');
    hold on; axis equal; xlim([-450 200]); ylim([-800 -300]); zlim([-200 150]);
    % Compute Euler angles at frame t
    Zp      = Vnorm_array3(Static.Pelvis.landmarks(:,1,1)-Static.Pelvis.landmarks(:,2,1));
    Yp      = Vnorm_array3(cross(Static.Pelvis.landmarks(:,1,1)-Static.Pelvis.landmarks(:,3,1),Static.Pelvis.landmarks(:,2,1)-Static.Pelvis.landmarks(:,3,1)));
    Xp      = Vnorm_array3(cross(Yp,Zp));
    Rp      = [Xp Yp Zp];
    Op      = (Static.Pelvis.landmarks(:,1,1)+Static.Pelvis.landmarks(:,2,1))/2;
    T_ics_p = [Rp Op; 0 0 0 1];
    Yf      = Vnorm_array3(Static.Femur.landmarks(:,3,t)-(Static.Femur.landmarks(:,1,t)+Static.Femur.landmarks(:,2,t))/2);
    Xf      = Vnorm_array3(cross(Static.Femur.landmarks(:,2,t)-Static.Femur.landmarks(:,3,t),Static.Femur.landmarks(:,1,t)-Static.Femur.landmarks(:,3,t)));
    Zf      = Vnorm_array3(cross(Xf,Yf));
    Rf      = [Xf Yf Zf];
    Of      = Static.Femur.landmarks(:,3,t);
    Rf      = [Xf Yf Zf];
    T_ics_f = [Rf Of; 0 0 0 1];
    Thip    = Mprod_array3(Tinv_array3(T_ics_p),T_ics_f);
    Euler   = rad2deg(R2mobileZXY_array3(Thip(1:3,1:3)));
    % Plot Euler angle values
    text(-430,-320,130,['e1: ',num2str(Euler(1)),' °']);
    text(-430,-320,100,['e2: ',num2str(Euler(2)),' °']);
    text(-430,-320,70,['e3: ',num2str(Euler(3)),' °']);
    % Plot landmarks and clusters
    plot3(Static.Pelvis.landmarks(1,:,1),...
          Static.Pelvis.landmarks(2,:,1),...
          Static.Pelvis.landmarks(3,:,1),...
          'Marker','.','Markersize',15,'Linestyle','none','Color','green');
    plot3(Static.Femur.cluster(1,:,1),...
          Static.Femur.cluster(2,:,1),...
          Static.Femur.cluster(3,:,1),...
          'Marker','.','Markersize',15,'Linestyle','none','Color','red');
    plot3(Static.Femur.landmarks(1,:,1),...
          Static.Femur.landmarks(2,:,1),...
          Static.Femur.landmarks(3,:,1),...
          'Marker','.','Markersize',15,'Linestyle','none','Color','green');
    plot3(Static.Femur.landmarks(1,:,t),...
          Static.Femur.landmarks(2,:,t),...
          Static.Femur.landmarks(3,:,t),...
          'Marker','.','Markersize',15,'Linestyle','none','Color','blue');
    plot3(permute(Static.Femur.landmarks(1,8,1:t),[3,1,2]),...
          permute(Static.Femur.landmarks(2,8,1:t),[3,1,2]),...
          permute(Static.Femur.landmarks(3,8,1:t),[3,1,2]),...
          'Marker','.','Markersize',5,'Linestyle','none','Color','black');
    % Plot segments
    line([Static.Pelvis.landmarks(1,1,1) Static.Pelvis.landmarks(1,2,1)], ...
         [Static.Pelvis.landmarks(2,1,1) Static.Pelvis.landmarks(2,2,1)], ...
         [Static.Pelvis.landmarks(3,1,1) Static.Pelvis.landmarks(3,2,1)], ...
         'Linewidth',2,'Color','green');
    line([Static.Pelvis.landmarks(1,3,1) Static.Pelvis.landmarks(1,4,1)], ...
         [Static.Pelvis.landmarks(2,3,1) Static.Pelvis.landmarks(2,4,1)], ...
         [Static.Pelvis.landmarks(3,3,1) Static.Pelvis.landmarks(3,4,1)], ...
         'Linewidth',2,'Color','green');
    line([Static.Pelvis.landmarks(1,1,1) Static.Pelvis.landmarks(1,3,1)], ...
         [Static.Pelvis.landmarks(2,1,1) Static.Pelvis.landmarks(2,3,1)], ...
         [Static.Pelvis.landmarks(3,1,1) Static.Pelvis.landmarks(3,3,1)], ...
         'Linewidth',2,'Color','green');
    line([Static.Pelvis.landmarks(1,2,1) Static.Pelvis.landmarks(1,4,1)], ...
         [Static.Pelvis.landmarks(2,2,1) Static.Pelvis.landmarks(2,4,1)], ...
         [Static.Pelvis.landmarks(3,2,1) Static.Pelvis.landmarks(3,4,1)], ...
         'Linewidth',2,'Color','green');
    line([Static.Femur.landmarks(1,1,1) Static.Femur.landmarks(1,2,1)], ...
         [Static.Femur.landmarks(2,1,1) Static.Femur.landmarks(2,2,1)], ...
         [Static.Femur.landmarks(3,1,1) Static.Femur.landmarks(3,2,1)], ...
         'Linewidth',2,'Color','green');
    line([Static.Femur.landmarks(1,1,1) Static.Femur.landmarks(1,3,1)], ...
         [Static.Femur.landmarks(2,1,1) Static.Femur.landmarks(2,3,1)], ...
         [Static.Femur.landmarks(3,1,1) Static.Femur.landmarks(3,3,1)], ...
         'Linewidth',2,'Color','green');
    line([Static.Femur.landmarks(1,2,1) Static.Femur.landmarks(1,3,1)], ...
         [Static.Femur.landmarks(2,2,1) Static.Femur.landmarks(2,3,1)], ...
         [Static.Femur.landmarks(3,2,1) Static.Femur.landmarks(3,3,1)], ...
         'Linewidth',2,'Color','green');
    line([Static.Femur.landmarks(1,1,t) Static.Femur.landmarks(1,2,t)], ...
         [Static.Femur.landmarks(2,1,t) Static.Femur.landmarks(2,2,t)], ...
         [Static.Femur.landmarks(3,1,t) Static.Femur.landmarks(3,2,t)], ...
         'Linewidth',2,'Color','blue');
    line([Static.Femur.landmarks(1,1,t) Static.Femur.landmarks(1,3,t)], ...
         [Static.Femur.landmarks(2,1,t) Static.Femur.landmarks(2,3,t)], ...
         [Static.Femur.landmarks(3,1,t) Static.Femur.landmarks(3,3,t)], ...
         'Linewidth',2,'Color','blue');
    line([Static.Femur.landmarks(1,2,t) Static.Femur.landmarks(1,3,t)], ...
         [Static.Femur.landmarks(2,2,t) Static.Femur.landmarks(2,3,t)], ...
         [Static.Femur.landmarks(3,2,t) Static.Femur.landmarks(3,3,t)], ...
         'Linewidth',2,'Color','blue');
    % Plot flange coordinate system
    quiver3(Flange.Ofl(1,1,t),Flange.Ofl(2,1,t),Flange.Ofl(3,1,t), ...
            Flange.Xfl(1,1,t),Flange.Xfl(2,1,t),Flange.Xfl(3,1,t),100,'red');
    quiver3(Flange.Ofl(1,1,t),Flange.Ofl(2,1,t),Flange.Ofl(3,1,t), ...
            Flange.Yfl(1,1,t),Flange.Yfl(2,1,t),Flange.Yfl(3,1,t),100,'green');
    quiver3(Flange.Ofl(1,1,t),Flange.Ofl(2,1,t),Flange.Ofl(3,1,t), ...
            Flange.Zfl(1,1,t),Flange.Zfl(2,1,t),Flange.Zfl(3,1,t),100,'blue'); 
    % Update plot
    if t < size(Static.Femur.landmarks,3)
        pause(0.01);
        cla;
        hold off;
    end
end
clearvars -except Folder Subject Robot T_qualisys_robot Stylus Calibration Functional Static Flange T_ics_hipref;