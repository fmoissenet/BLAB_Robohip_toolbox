% ADD HEADER

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
Folder.data    = 'C:\Users\moissene\OneDrive - unige.ch\2022 - ROBOHIP\Development\Biomécanique\calibrage_TRIPTERON\';
Folder.dep     = [Folder.toolbox,'dependencies\'];
addpath(Folder.toolbox);
addpath(genpath(Folder.dep));
cd(Folder.data);
figure; hold on; axis equal;

% -------------------------------------------------------------------------
% GENERATE A TRIPTERON DEFINITION FILE
% -------------------------------------------------------------------------
% Get LFRAME markers
c3dFile = 'LFRAME01.c3d';
btkFile = btkReadAcquisition(c3dFile);
Marker  = btkGetMarkers(btkFile);
TRIPTERON_p1 = mean(Marker.TRIPTERON_p1);
TRIPTERON_p2 = mean(Marker.TRIPTERON_p2);
TRIPTERON_p3 = mean(Marker.TRIPTERON_p3);
TRIPTERON_p4 = mean(Marker.TRIPTERON_p4);
% Initialise new btk file
newbtkFile = btkNewAcquisition();
btkSetFrequency(newbtkFile,100);
btkSetFrameNumber(newbtkFile,1);
btkSetPointsUnit(newbtkFile,'marker','mm');
btkAppendPoint(newbtkFile,'marker','TRIPTERON_p1',TRIPTERON_p1,0,'');
btkAppendPoint(newbtkFile,'marker','TRIPTERON_p2',TRIPTERON_p2,0,'');
btkAppendPoint(newbtkFile,'marker','TRIPTERON_p3',TRIPTERON_p3,0,'');
btkAppendPoint(newbtkFile,'marker','TRIPTERON_p4',TRIPTERON_p4,0,'');

% -------------------------------------------------------------------------
% LINMOT X
% -------------------------------------------------------------------------
clearvars -except Folder Tripteron newbtkFile;
% Axis estimation by least-square plan approach
c3dFile = 'LINMOTX01.c3d';
btkFile = btkReadAcquisition(c3dFile);
Marker  = btkGetMarkers(btkFile);
pointCloud1 = Marker.STYLUSa_TIP;
[x1,a1] = lsplane(pointCloud1);
% Axis estimation by least-square cylinder approach
c3dFile = 'LINMOTX02.c3d';
btkFile = btkReadAcquisition(c3dFile);
Marker  = btkGetMarkers(btkFile);
pointCloud2 = Marker.STYLUSa_TIP;
[x2,a2] = lscylinder(pointCloud2,x1,a1,1,1,0.025);
% Error between the two estimated axes (3d angle, deg)
errorAngle = rad2deg(atan2(norm(cross(a1,a2)),dot(a1,a2)))
% Store the average axis vector
Tripteron.Axis.X = -(a1+a2)/2;
% Generate two virtual markers to be defined in QTM to define a Tripteron
% 6DOF = related to the LFRAME 6DOF fixed on the table = if the table
% moves, the Tripteron is assumed not to have moved related to this 6DOF =
% no need to redefine Tripteron axes except if Linmots move or LFRAME moves
origin   = mean(Marker.TRIPTERON_p2);
LinmotX1 = origin+100*Tripteron.Axis.X';
LinmotX2 = origin-100*Tripteron.Axis.X';
% Store virtual markers in btkFile
btkAppendPoint(newbtkFile,'marker','LinmotX1',LinmotX1,0,'Linmot+X');
btkAppendPoint(newbtkFile,'marker','LinmotX2',LinmotX2,0,'Linmot-X');
% Plot
plot3(pointCloud1(:,1),pointCloud1(:,2),pointCloud1(:,3),'Marker','.','MarkerSize',10,'Color','black','LineStyle','none');
quiver3(x1(1),x1(2),x1(3),a1(1),a1(2),a1(3),50,'red');
plot3(pointCloud2(:,1),pointCloud2(:,2),pointCloud2(:,3),'Marker','.','MarkerSize',10,'Color','black','LineStyle','none');
quiver3(x1(1),x1(2),x1(3),a2(1),a2(2),a2(3),50,'blue');
plot3(origin(1),origin(2),origin(3),'Marker','o','MarkerSize',5,'Color','black','LineStyle','none');
quiver3(origin(1),origin(2),origin(3),Tripteron.Axis.X(1),Tripteron.Axis.X(2),Tripteron.Axis.X(3),100,'red');
plot3(LinmotX1(1),LinmotX1(2),LinmotX1(3),'Marker','o','MarkerSize',5,'Color','red','LineStyle','none');
plot3(LinmotX2(1),LinmotX2(2),LinmotX2(3),'Marker','o','MarkerSize',5,'Color','red','LineStyle','none');

% -------------------------------------------------------------------------
% LINMOT Y
% -------------------------------------------------------------------------
clearvars -except Folder Tripteron newbtkFile;
% Axis estimation by least-square plan approach
c3dFile = 'LINMOTY01.c3d';
btkFile = btkReadAcquisition(c3dFile);
Marker  = btkGetMarkers(btkFile);
pointCloud1 = Marker.STYLUSa_TIP;
[x1,a1] = lsplane(pointCloud1);
% Axis estimation by least-square cylinder approach
c3dFile = 'LINMOTY02.c3d';
btkFile = btkReadAcquisition(c3dFile);
Marker  = btkGetMarkers(btkFile);
pointCloud2 = Marker.STYLUSa_TIP;
[x2,a2] = lscylinder(pointCloud2,x1,a1,1,1,0.025);
% Error between the two estimated axes (3d angle, deg)
errorAngle = rad2deg(atan2(norm(cross(a1,a2)),dot(a1,a2)))
% Store the average axis vector
Tripteron.Axis.Y = (a1+a2)/2;
% Generate two virtual markers to be defined in QTM to define a Tripteron
% 6DOF = related to the LFRAME 6DOF fixed on the table = if the table
% moves, the Tripteron is assumed not to have moved related to this 6DOF =
% no need to redefine Tripteron axes except if Linmots move or LFRAME moves
origin   = mean(Marker.TRIPTERON_p2);
LinmotY1 = origin+100*Tripteron.Axis.Y';
LinmotY2 = origin-100*Tripteron.Axis.Y';
% Store virtual markers in btkFile
btkAppendPoint(newbtkFile,'marker','LinmotY1',LinmotY1,0,'Linmot+Y');
btkAppendPoint(newbtkFile,'marker','LinmotY2',LinmotY2,0,'Linmot-Y');
% Plot
plot3(pointCloud1(:,1),pointCloud1(:,2),pointCloud1(:,3),'Marker','.','MarkerSize',10,'Color','black','LineStyle','none');
quiver3(x1(1),x1(2),x1(3),a1(1),a1(2),a1(3),50,'red');
plot3(pointCloud2(:,1),pointCloud2(:,2),pointCloud2(:,3),'Marker','.','MarkerSize',10,'Color','black','LineStyle','none');
quiver3(x1(1),x1(2),x1(3),a2(1),a2(2),a2(3),50,'blue');
quiver3(origin(1),origin(2),origin(3),Tripteron.Axis.Y(1),Tripteron.Axis.Y(2),Tripteron.Axis.Y(3),100,'green');
plot3(LinmotY1(1),LinmotY1(2),LinmotY1(3),'Marker','o','MarkerSize',5,'Color','green','LineStyle','none');
plot3(LinmotY2(1),LinmotY2(2),LinmotY2(3),'Marker','o','MarkerSize',5,'Color','green','LineStyle','none');

% -------------------------------------------------------------------------
% LINMOT Z
% -------------------------------------------------------------------------
clearvars -except Folder Tripteron newbtkFile;
% Axis estimation by least-square plan approach
c3dFile = 'LINMOTZ01.c3d';
btkFile = btkReadAcquisition(c3dFile);
Marker  = btkGetMarkers(btkFile);
pointCloud1 = Marker.STYLUSa_TIP;
[x1,a1] = lsplane(pointCloud1);
% Axis estimation by least-square cylinder approach
c3dFile = 'LINMOTZ02.c3d';
btkFile = btkReadAcquisition(c3dFile);
Marker  = btkGetMarkers(btkFile);
pointCloud2 = Marker.STYLUSa_TIP;
[x2,a2] = lscylinder(pointCloud2,x1,a1,1,1,0.025);
% Error between the two estimated axes (3d angle, deg)
errorAngle = rad2deg(atan2(norm(cross(a1,a2)),dot(a1,a2)))
% Store the average axis vector
Tripteron.Axis.Z = (a1+a2)/2;
% Generate two virtual markers to be defined in QTM to define a Tripteron
% 6DOF = related to the LFRAME 6DOF fixed on the table = if the table
% moves, the Tripteron is assumed not to have moved related to this 6DOF =
% no need to redefine Tripteron axes except if Linmots move or LFRAME moves
origin   = mean(Marker.TRIPTERON_p2);
LinmotZ1 = origin+100*Tripteron.Axis.Z';
LinmotZ2 = origin-100*Tripteron.Axis.Z';
% Store virtual markers in btkFile
btkAppendPoint(newbtkFile,'marker','LinmotZ1',LinmotZ1,0,'Linmot+Z');
btkAppendPoint(newbtkFile,'marker','LinmotZ2',LinmotZ2,0,'Linmot-Z');
% Plot
plot3(pointCloud1(:,1),pointCloud1(:,2),pointCloud1(:,3),'Marker','.','MarkerSize',10,'Color','black','LineStyle','none');
quiver3(x1(1),x1(2),x1(3),a1(1),a1(2),a1(3),50,'red');
plot3(pointCloud2(:,1),pointCloud2(:,2),pointCloud2(:,3),'Marker','.','MarkerSize',10,'Color','black','LineStyle','none');
quiver3(x1(1),x1(2),x1(3),a2(1),a2(2),a2(3),50,'blue');
quiver3(origin(1),origin(2),origin(3),Tripteron.Axis.Z(1),Tripteron.Axis.Z(2),Tripteron.Axis.Z(3),100,'blue');
plot3(LinmotZ1(1),LinmotZ1(2),LinmotZ1(3),'Marker','o','MarkerSize',5,'Color','blue','LineStyle','none');
plot3(LinmotZ2(1),LinmotZ2(2),LinmotZ2(3),'Marker','o','MarkerSize',5,'Color','blue','LineStyle','none');

% -------------------------------------------------------------------------
% DEFINE TRIPTERON ROTATION MATRIX AND QUATERNION
% -------------------------------------------------------------------------
Tripteron.Orientation.R = [Tripteron.Axis.X Tripteron.Axis.Y Tripteron.Axis.Z];
Tripteron.Orientation.Q = rotMat2quatern(Tripteron.Orientation.R);

% -------------------------------------------------------------------------
% GENERATE A TRIPTERON DEFINITION FILE
% -------------------------------------------------------------------------
btkWriteAcquisition(newbtkFile,'TRIPTERON01.c3d');