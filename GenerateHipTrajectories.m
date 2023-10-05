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
Folder.toolbox = 'C:\Users\moissene\OneDrive - unige.ch\2022 - ROBOHIP\Development\Biomécanique\BLAB_Robohip_toolbox\';
Folder.data    = 'C:\Users\moissene\OneDrive - unige.ch\2022 - ROBOHIP\Data\Tests\Essais_20230913\';
Folder.export  = 'C:\Users\moissene\OneDrive - unige.ch\2022 - ROBOHIP\Data\Tests\Essais_20230913\';
Folder.dep     = [Folder.toolbox,'dependencies\'];
addpath(Folder.toolbox);
addpath(genpath(Folder.dep));
cd(Folder.data);

% -------------------------------------------------------------------------
% SET SPECIMEN INFO
% -------------------------------------------------------------------------
Specimen.id   = 'RH000';
Specimen.side = 'R'; % R or L

% -------------------------------------------------------------------------
% SET ROBOT PARAMETERS
% -------------------------------------------------------------------------
Robot.ThicknessBotaPlate = 10; % Thickness of the Bota sensor plate (mm)

%% ------------------------------------------------------------------------
% TO BE DONE BEFORE
% -------------------------------------------------------------------------
% a- The robot calibration must have been done, with results applied to QTM

%% ------------------------------------------------------------------------
% 1. DEFINE PELVIS CLUSTER TECHNICAL FRAME
% -------------------------------------------------------------------------
% - In QTM, record the position of the pelvis cluster markers PELVIS_ci and  
%   point anatomical landmarks PELVIS_RASI, PELVIS_LASI, PELVIS_RPSI,
%   PELVIS_LPSI (before fixing the pelvis on the TRIPTERON robot)
% - Store records in 6DOFs_PELVIS_01.qtm
% - Create two point trajectories (virtual markers) defined as the middle 
%   of PELVIS_RASI and PELVIS_LASI, called PELVIS_MASI, and as the middle 
%   of PELVIS_RPSI and PELVIS_LPSI, called PELVIS_MPSI
% - Define a new 6DOFs object PELVIS_cluster with PELVIS_ci, PELVIS_RASI,
%   PELVIS_LASI, PELVIS_RPSI, PELVIS_LPSI, PELVIS_MASI, PELVIS_MPSI
% - Translate the 6DOFs object to PELVIS_c1
% - Rotate the 6DOFs object as X from c3 to c1 and Y parallel to a line 
%   from c2 to c1
% - Recompute 6DOF and save file

%% ------------------------------------------------------------------------
% 2. DEFINE RFEMUR CLUSTER TECHNICAL FRAME
% -------------------------------------------------------------------------
% - In QTM, record the position of the right femur cluster markers 
%   RFEMUR_ci and point anatomical landmarks RFEMUR_RFLE and RFEMUR_RFME
% - Store records in 6DOFs_RFEMUR_01.qtm
% - Create one point trajectory (virtual marker) defined as the middle of 
%   RFEMUR_RFLE and RFEMUR_RFME, called RFEMUR_RKJC
% - Define a new 6DOFs object RFEMUR_cluster with RFEMUR_ci, RFEMUR_RFLE,
%   RFEMUR_RFME, RFEMUR_RKJC
% - Translate the 6DOFs object to RFEMUR_c1
% - Rotate the 6DOFs object as X from c3 to c1 and Y parallel to a line 
%   from c2 to c1
% - Recompute 6DOF and save file

%% ------------------------------------------------------------------------
% 3. COMPUTE THE HIP JOINT CENTRES
% -------------------------------------------------------------------------
% - In QTM, record right femur circumduction/rotations to allow right hip  
%   joint centre computation
% - Compute 6DOFs
% - Store records in JC_RHIP.qtm
% - Export records in JC_RHIP.c3d
% - Do the same for the left femur (JC_LHIP.qtm and JC_LHIP.c3d)
% -------------------------------------------------------------------------

% Clear workspace
clearvars -except Folder Specimen Robot;

% Load C3D file 
[Marker,Event,firstFrame,nMarker,fMarker,markerNames] = LoadC3DFile('JC_RHIP.c3d');

% Define a technical coordinate system related to the pelvis cluster as 
% defined in QTM
Op = Marker.PELVIS_c1;
Xp = Vnorm_array3(Marker.PELVIS_c1-Marker.PELVIS_c3);
Yp = Vnorm_array3(Marker.PELVIS_c1-Marker.PELVIS_c2);
Zp = Vnorm_array3(cross(Xp,Yp));
Yp = Vnorm_array3(cross(Zp,Xp));
Tp = [Xp Yp Zp Op; zeros(1,3,nMarker) ones(1,1,nMarker)];

% Define a technical coordinate system related to the rfemur cluster as 
% defined in QTM
Orf = Marker.RFEMUR_c1;
Xrf = Vnorm_array3(Marker.RFEMUR_c1-Marker.RFEMUR_c3);
Yrf = Vnorm_array3(Marker.RFEMUR_c1-Marker.RFEMUR_c2);
Zrf = Vnorm_array3(cross(Xrf,Yrf));
Yrf = Vnorm_array3(cross(Zrf,Xrf));
Trf = [Xrf Yrf Zrf Orf; zeros(1,3,nMarker) ones(1,1,nMarker)];

% Compute the local position of RHJC in pelvis cluster coordinate system,
% and in right femur cluster coordinate system
[rC,rCsi,rCsj,Residual] = SCoRE_array3(Tp,Trf)

% Clear workspace
clearvars -except Folder Subject Robot;

% Load C3D file 
[Marker,Event,firstFrame,nMarker,fMarker,markerNames] = LoadC3DFile('JC_LHIP.c3d');

% Define a technical coordinate system related to the pelvis cluster as 
% defined in QTM
Op = Marker.PELVIS_c1;
Xp = Vnorm_array3(Marker.PELVIS_c1-Marker.PELVIS_c3);
Yp = Vnorm_array3(Marker.PELVIS_c1-Marker.PELVIS_c2);
Zp = Vnorm_array3(cross(Xp,Yp));
Yp = Vnorm_array3(cross(Zp,Xp));
Tp = [Xp Yp Zp Op; zeros(1,3,nMarker) ones(1,1,nMarker)];

% Define a technical coordinate system related to the rfemur cluster as 
% defined in QTM
Olf = Marker.LFEMUR_c1;
Xlf = Vnorm_array3(Marker.LFEMUR_c1-Marker.LFEMUR_c3);
Ylf = Vnorm_array3(Marker.LFEMUR_c1-Marker.LFEMUR_c2);
Zlf = Vnorm_array3(cross(Xlf,Ylf));
Ylf = Vnorm_array3(cross(Zlf,Xlf));
Tlf = [Xlf Ylf Zlf Olf; zeros(1,3,nMarker) ones(1,1,nMarker)];

% Compute the local position of RHJC in pelvis cluster coordinate system
[rC,rCsi,rCsj,Residual] = SCoRE_array3(Tp,Tlf)

%% ------------------------------------------------------------------------
% 4. DEFINE PELVIS ANATOMICAL FRAME
% -------------------------------------------------------------------------
% - In QTM, open 6DOFs_PELVIS_01.qtm
% - Report the values of RHJC and LHJC as virtual points of the
%   PELVIS_cluster 6DOFs object: PELVIS_RHJC and PELVIS_LHJC
% - Recompute 6DOFs
% - Add a new trajectory defined as the average of PELVIS_RHJC and
%   PELVIS_LHJC, called PELVIS_MHJC
% - Add PELVIS_MHJC to PELVIS_cluster 6DOFs object
% - Translate the 6DOFs object to PELVIS_MHJC
% - Rotate the 6DOFs object as Z from PELVIS_LHJC to PELVIS_RHJC and X
%   parallel to a line defined from PELVIS_MPSI to PELVIS_MASI
% - Rename PELVIS_cluster 6DOFs object as PELVIS_anatomical
% - Recompute 6DOF and save file

%% ------------------------------------------------------------------------
% 5. ADJUST THE PELVIS POSITION AND ORIENTATION
% -------------------------------------------------------------------------
% - Using PELVIS_cluster 6DOFs object position and orientation provided in
%   real time by QTM, align Zp with Y, and place PELVIS_MHJC at almost the
%   following position: 

%% ------------------------------------------------------------------------
% 6. DEFINE THE RFEMUR COORDINATE SYSTEM
% -------------------------------------------------------------------------
% - In QTM, open 6DOFs_RFEMUR_01.qtm
% - Report the values of RHJC as a virtual point of the RFEMUR_cluster  
%   6DOFs object: RFEMUR_RHJC
% - Add RFEMUR_RHJC to RFEMUR_cluster 6DOFs object
% - Translate the 6DOFs object to RFEMUR_RHJC
% - Rotate the 6DOFs object as Y from RFEMUR_RKJC to RFEMUR_RHJC and Z
%   parallel to a line defined from RFEMUR_RFME to RFEMUR_RFLE
% - Recompute 6DOF and save file

%% ------------------------------------------------------------------------
% 7. CONNECT ROBOT TO RFEMUR
% -------------------------------------------------------------------------
% - Pre-positioned the robot as follow:
%   A1: -145° / A2: +60° / A3: +25° / A4: -75° / A5: +50° / A6: +110°
% - Take care also to place A7 at +65°
% - Adapt the previously defined joint positions to the need
% - Once connected, save the position as initial position in the robot
%   controller unit (Matlab)
% - Save initial pose STATIC_RFEMUR_01.c3d

%% ------------------------------------------------------------------------
% 8. GENERATE RFEMUR REQUESTED MOTIONS
% -------------------------------------------------------------------------

% Clear workspace
clearvars -except Folder Specimen Robot;
% clc;

% Load C3D file 
[Marker,Event,firstFrame,nMarker,fMarker,markerNames] = LoadC3DFile('STATIC_RFEMUR_01.c3d');

% Compute mean marker position across all frames
for imarker = 1:size(markerNames,1)
    Marker.(markerNames{imarker}) = mean(Marker.(markerNames{imarker}),3,'omitnan');
end

% Define PELVIS coordinate system
Op      = Marker.PELVIS_MHJC; % Differs from Wu et al. 2002, needed to set the pelvis in space during the protocol
Zp      = Vnorm_array3(Marker.PELVIS_RHJC-Marker.PELVIS_LHJC);
Yp      = Vnorm_array3(cross(Marker.PELVIS_RASI-Marker.PELVIS_RPSI,Marker.PELVIS_LASI-Marker.PELVIS_LPSI));
Xp      = Vnorm_array3(cross(Yp,Zp));
Rp      = [Xp Yp Zp];
T_ics_p = [Rp Op; 0 0 0 1];

% Define RFEMUR coordinate system
Of      = Marker.RFEMUR_RHJC;
Yf      = Vnorm_array3(Marker.RFEMUR_RHJC-Marker.RFEMUR_RKJC);
Xf      = Vnorm_array3(cross(Marker.RFEMUR_RFLE-Marker.RFEMUR_RHJC,Marker.RFEMUR_RFME-Marker.RFEMUR_RHJC));
Zf      = Vnorm_array3(cross(Xf,Yf));
Rf      = [Xf Yf Zf];
T_ics_f = [Rf Of; 0 0 0 1];

% Compute angle difference between pelvis and rfemur coordinate systems
% Express Euler angle using the ZXY sequence
% i.e. which angles to apply to the femur to be aligned with the pelvis
temp  = Mprod_array3(Tinv_array3(T_ics_p),T_ics_f);
Euler = rad2deg(R2fixedYXZ_array3(temp(1:3,1:3,:)));
disp(['Initial alignment differences between femur and pelvis (ZXY): ',num2str(Euler),'°']);

% Define KUKA_flange coordinate system
Ofl      = Marker.KUKA_flange_04;
Xfl      = Vnorm_array3(Marker.KUKA_flange_01-Marker.KUKA_flange_03);
Yfl      = Vnorm_array3(Marker.KUKA_flange_04-Marker.KUKA_flange_02);
Zfl      = Vnorm_array3(cross(Xfl,Yfl)); 
Xfl      = Vnorm_array3(cross(Yfl,Zfl));
Ofl      = Ofl-5*Zfl; % 5 is the half height of the BOTA fixation plate
Rfl      = [Xfl Yfl Zfl];
T_ics_fl = [Rfl Ofl; 0 0 0 1];

% Store rigid transformation between the flange and the femur
T_f_fl = Mprod_array3(Minv_array3(T_ics_f),T_ics_fl);

% Initialise motion generation parameters
start     = 0;
stop      = 0;
order     = 0;
precision = 0.05; % deg

% Set motion list
% Motion name | Rotation axis | Angle values
% One planning file for the KUKA robot is written for each motion
% Angles are cumulative (i.e. after 30° of flexion, only 30° needed to
% reach 60°)
% motionList = {...
%               'anatomicAlignementZ' 'Z' 0:sign(Euler(1))*precision:Euler(1); ...
%               'anatomicAlignementX' 'X' 0:sign(Euler(2))*precision:Euler(2); ...
%               'anatomicAlignementY' 'Y' 0:sign(Euler(3))*precision:Euler(3); ...
%               'extension30'         'Z' 0:1*precision:30; ...
%               'intRotation90a'      'Y' 0:-1*precision:-90; ... % go
%               'intRotation90b'      'Y' 0:1*precision:90; ... % return
%               'extRotation90a'      'Y' 0:1*precision:90; ... % go
%               'extRotation90b'      'Y' 0:-1*precision:-90; ... % return
%               'extension20'         'Z' 0:-1*precision:-10; ...
%               'intRotation90a'      'Y' 0:-1*precision:-90; ... % go
%               'intRotation90b'      'Y' 0:1*precision:90; ... % return
%               'extRotation90a'      'Y' 0:1*precision:90; ... % go
%               'extRotation90b'      'Y' 0:-1*precision:-90; ... % return
%               'flexion0'            'Z' 0:-1*precision:-20; ...
%               'intRotation90a'      'Y' 0:-1*precision:-90; ... % go
%               'intRotation90b'      'Y' 0:1*precision:90; ... % return
%               'extRotation90a'      'Y' 0:1*precision:90; ... % go
%               'extRotation90b'      'Y' 0:-1*precision:-90; ... % return
%               'flexion20'           'Z' 0:-1*precision:-20; ...
%               'intRotation90a'      'Y' 0:-1*precision:-90; ... % go
%               'intRotation90b'      'Y' 0:1*precision:90; ... % return
%               'extRotation90a'      'Y' 0:1*precision:90; ... % go
%               'extRotation90b'      'Y' 0:-1*precision:-90; ... % return
%               'flexion40'           'Z' 0:-1*precision:-20; ...
%               'intRotation90a'      'Y' 0:-1*precision:-90; ... % go
%               'intRotation90b'      'Y' 0:1*precision:90; ... % return
%               'extRotation90a'      'Y' 0:1*precision:90; ... % go
%               'extRotation90b'      'Y' 0:-1*precision:-90; ... % return
%               'flexion60'           'Z' 0:-1*precision:-20; ...
%               'intRotation90a'      'Y' 0:-1*precision:-90; ... % go
%               'intRotation90b'      'Y' 0:1*precision:90; ... % return
%               'extRotation90a'      'Y' 0:1*precision:90; ... % go
%               'extRotation90b'      'Y' 0:-1*precision:-90; ... % return
%               'flexion80'           'Z' 0:-1*precision:-20; ...
%               'intRotation90a'      'Y' 0:-1*precision:-90; ... % go
%               'intRotation90b'      'Y' 0:1*precision:90; ... % return
%               'extRotation90a'      'Y' 0:1*precision:90; ... % go
%               'extRotation90b'      'Y' 0:-1*precision:-90; ... % return
%               'flexion100'          'Z' 0:-1*precision:-20; ...
%               'intRotation90a'      'Y' 0:-1*precision:-90; ... % go
%               'intRotation90b'      'Y' 0:1*precision:90; ... % return
%               'extRotation90a'      'Y' 0:1*precision:90; ... % go
%               'extRotation90b'      'Y' 0:-1*precision:-90; ... % return
%               'flexion120'          'Z' 0:-1*precision:-20; ...
%               'intRotation90a'      'Y' 0:-1*precision:-90; ... % go
%               'intRotation90b'      'Y' 0:1*precision:90; ... % return
%               'extRotation90a'      'Y' 0:1*precision:90; ... % go
%               'extRotation90b'      'Y' 0:-1*precision:-90; ... % return
%               'flexion0'            'Z' 0:1*precision:120; ...
%               };
motionList = {...
              'anatomicAlignementY' 'Y' 0:sign(Euler(1))*precision:Euler(1); ...
              'anatomicAlignementX' 'X' 0:sign(Euler(2))*precision:Euler(2); ...
              'anatomicAlignementZ' 'Z' 0:sign(Euler(3))*precision:Euler(3); ...
              'extension30'         'Z' 0:1*precision:30; ...
              'intRotation45a'      'Y' 0:-1*precision:-45; ... % go
              'intRotation45b'      'Y' 0:1*precision:45; ... % return
              'extRotation45a'      'Y' 0:1*precision:45; ... % go
              'extRotation45b'      'Y' 0:-1*precision:-45; ... % return
              'extension20'         'Z' 0:-1*precision:-10; ...
              'intRotation45a'      'Y' 0:-1*precision:-45; ... % go
              'intRotation45b'      'Y' 0:1*precision:45; ... % return
              'extRotation45a'      'Y' 0:1*precision:45; ... % go
              'extRotation45b'      'Y' 0:-1*precision:-45; ... % return
              'flexion0'            'Z' 0:-1*precision:-20; ...
              'intRotation45a'      'Y' 0:-1*precision:-45; ... % go
              'intRotation45b'      'Y' 0:1*precision:45; ... % return
              'extRotation45a'      'Y' 0:1*precision:45; ... % go
              'extRotation45b'      'Y' 0:-1*precision:-45; ... % return
              'flexion20'           'Z' 0:-1*precision:-20; ...
              'intRotation45a'      'Y' 0:-1*precision:-45; ... % go
              'intRotation45b'      'Y' 0:1*precision:45; ... % return
              'extRotation45a'      'Y' 0:1*precision:45; ... % go
              'extRotation45b'      'Y' 0:-1*precision:-45; ... % return
              'flexion40'           'Z' 0:-1*precision:-20; ...
              'intRotation45a'      'Y' 0:-1*precision:-45; ... % go
              'intRotation45b'      'Y' 0:1*precision:45; ... % return
              'extRotation45a'      'Y' 0:1*precision:45; ... % go
              'extRotation45b'      'Y' 0:-1*precision:-45; ... % return
              'flexion60'           'Z' 0:-1*precision:-20; ...
              'intRotation45a'      'Y' 0:-1*precision:-45; ... % go
              'intRotation45b'      'Y' 0:1*precision:45; ... % return
              'extRotation45a'      'Y' 0:1*precision:45; ... % go
              'extRotation45b'      'Y' 0:-1*precision:-45; ... % return
              'flexion0'            'Z' 0:1*precision:60; ... % return
              };

% Generate requested motions
% Update the T_ics_f matrix
% Erase the previous version of the T_ics_fl matrix
for imotion = 1:size(motionList,1)
    [T_ics_f,T_ics_fl,start,stop] = AddMotion(T_ics_p,Specimen,Folder,motionList{imotion,3},motionList{imotion,2},T_ics_f,T_ics_fl,T_f_fl,stop,imotion,motionList{imotion,1});
    % Check angular values
    temp = Mprod_array3(Tinv_array3(T_ics_p),T_ics_f(:,:,end));
    R    = temp(1:3,1:3);
    % Plot the resulting angular values
    Euler2 = rad2deg(R2mobileZXY_array3(R)); % Difference with target for each angle should be <1°
    disp(['Angle values around each axis of the hip JCS (ZXY): ',num2str(round(Euler2(1),1)),' | ',num2str(round(Euler2(2),1)),' | ',num2str(round(Euler2(3),1)),'°']);
    clear temp R x y z error x_ort y_ort z_ortEuler2;    
end

%% Plot (ONLY FOR TEST)
localRFLE = Mprod_array3(Minv_array3(T_ics_f(:,:,1)),[Marker.RFEMUR_RFLE;1]);
localRFME = Mprod_array3(Minv_array3(T_ics_f(:,:,1)),[Marker.RFEMUR_RFME;1]);
localRKJC = Mprod_array3(Minv_array3(T_ics_f(:,:,1)),[Marker.RFEMUR_RKJC;1]);
Marker.RFEMUR_RFLE = Mprod_array3(T_ics_f,repmat(localRFLE,[1 1 stop+1]));
Marker.RFEMUR_RFME = Mprod_array3(T_ics_f,repmat(localRFME,[1 1 stop+1]));
Marker.RFEMUR_RKJC = Mprod_array3(T_ics_f,repmat(localRKJC,[1 1 stop+1]));

figure(1);
for iframe = 1:10:stop
    % Markers
    plot3(Marker.PELVIS_RASI(1,:,1),Marker.PELVIS_RASI(2,:,1),Marker.PELVIS_RASI(3,:,1),'Marker','.','MarkerSize',20,'Color','black');
    hold on; axis equal;
    plot3(Marker.PELVIS_LASI(1,:,1),Marker.PELVIS_LASI(2,:,1),Marker.PELVIS_LASI(3,:,1),'Marker','.','MarkerSize',20,'Color','black');
    plot3(Marker.PELVIS_RPSI(1,:,1),Marker.PELVIS_RPSI(2,:,1),Marker.PELVIS_RPSI(3,:,1),'Marker','.','MarkerSize',20,'Color','black');
    plot3(Marker.PELVIS_LPSI(1,:,1),Marker.PELVIS_LPSI(2,:,1),Marker.PELVIS_LPSI(3,:,1),'Marker','.','MarkerSize',20,'Color','black');
    plot3(Marker.PELVIS_RHJC(1,:,1),Marker.PELVIS_RHJC(2,:,1),Marker.PELVIS_RHJC(3,:,1),'Marker','.','MarkerSize',20,'Color','red');
    plot3(Marker.RFEMUR_RFLE(1,:,iframe),Marker.RFEMUR_RFLE(2,:,iframe),Marker.RFEMUR_RFLE(3,:,iframe),'Marker','.','MarkerSize',20,'Color','black');
    plot3(Marker.RFEMUR_RFME(1,:,iframe),Marker.RFEMUR_RFME(2,:,iframe),Marker.RFEMUR_RFME(3,:,iframe),'Marker','.','MarkerSize',20,'Color','black');
    plot3(Marker.RFEMUR_RKJC(1,:,iframe),Marker.RFEMUR_RKJC(2,:,iframe),Marker.RFEMUR_RKJC(3,:,iframe),'Marker','.','MarkerSize',20,'Color','red');
    % Segments
    line([Marker.RFEMUR_RKJC(1,:,iframe) Marker.PELVIS_RHJC(1,:,1)], ...
         [Marker.RFEMUR_RKJC(2,:,iframe) Marker.PELVIS_RHJC(2,:,1)], ...
         [Marker.RFEMUR_RKJC(3,:,iframe) Marker.PELVIS_RHJC(3,:,1)], ...
         'Linewidth',3,'Color','black');
    line([Marker.PELVIS_RASI(1,:,1) Marker.PELVIS_LASI(1,:,1)], ...
         [Marker.PELVIS_RASI(2,:,1) Marker.PELVIS_LASI(2,:,1)], ...
         [Marker.PELVIS_RASI(3,:,1) Marker.PELVIS_LASI(3,:,1)], ...
         'Linewidth',3,'Color','black');
    line([Marker.PELVIS_RASI(1,:,1) Marker.PELVIS_RPSI(1,:,1)], ...
         [Marker.PELVIS_RASI(2,:,1) Marker.PELVIS_RPSI(2,:,1)], ...
         [Marker.PELVIS_RASI(3,:,1) Marker.PELVIS_RPSI(3,:,1)], ...
         'Linewidth',3,'Color','black');
    line([Marker.PELVIS_LASI(1,:,1) Marker.PELVIS_LPSI(1,:,1)], ...
         [Marker.PELVIS_LASI(2,:,1) Marker.PELVIS_LPSI(2,:,1)], ...
         [Marker.PELVIS_LASI(3,:,1) Marker.PELVIS_LPSI(3,:,1)], ...
         'Linewidth',3,'Color','black');
    line([Marker.PELVIS_RPSI(1,:,1) Marker.PELVIS_LPSI(1,:,1)], ...
         [Marker.PELVIS_RPSI(2,:,1) Marker.PELVIS_LPSI(2,:,1)], ...
         [Marker.PELVIS_RPSI(3,:,1) Marker.PELVIS_LPSI(3,:,1)], ...
         'Linewidth',3,'Color','black');
    % Coordinate systems
    quiver3(T_ics_p(1,4,1),T_ics_p(2,4,1),T_ics_p(3,4,1), ...
            T_ics_p(1,1,1),T_ics_p(2,1,1),T_ics_p(3,1,1), ...
            50,'red');
    quiver3(T_ics_p(1,4,1),T_ics_p(2,4,1),T_ics_p(3,4,1), ...
            T_ics_p(1,2,1),T_ics_p(2,2,1),T_ics_p(3,2,1), ...
            50,'green');
    quiver3(T_ics_p(1,4,1),T_ics_p(2,4,1),T_ics_p(3,4,1), ...
            T_ics_p(1,3,1),T_ics_p(2,3,1),T_ics_p(3,3,1), ...
            50,'blue');
    quiver3(T_ics_f(1,4,iframe),T_ics_f(2,4,iframe),T_ics_f(3,4,iframe), ...
            T_ics_f(1,1,iframe),T_ics_f(2,1,iframe),T_ics_f(3,1,iframe), ...
            50,'red');
    quiver3(T_ics_f(1,4,iframe),T_ics_f(2,4,iframe),T_ics_f(3,4,iframe), ...
            T_ics_f(1,2,iframe),T_ics_f(2,2,iframe),T_ics_f(3,2,iframe), ...
            50,'green');
    quiver3(T_ics_f(1,4,iframe),T_ics_f(2,4,iframe),T_ics_f(3,4,iframe), ...
            T_ics_f(1,3,iframe),T_ics_f(2,3,iframe),T_ics_f(3,3,iframe), ...
            50,'blue');
    quiver3(T_ics_fl(1,4,iframe),T_ics_fl(2,4,iframe),T_ics_fl(3,4,iframe), ...
            T_ics_fl(1,1,iframe),T_ics_fl(2,1,iframe),T_ics_fl(3,1,iframe), ...
            50,'red');
    quiver3(T_ics_fl(1,4,iframe),T_ics_fl(2,4,iframe),T_ics_fl(3,4,iframe), ...
            T_ics_fl(1,2,iframe),T_ics_fl(2,2,iframe),T_ics_fl(3,2,iframe), ...
            50,'green');
    quiver3(T_ics_fl(1,4,iframe),T_ics_fl(2,4,iframe),T_ics_fl(3,4,iframe), ...
            T_ics_fl(1,3,iframe),T_ics_fl(2,3,iframe),T_ics_fl(3,3,iframe), ...
            50,'blue');
    % Clean view
    clear gca;
    pause(0.01);
    hold off;
end