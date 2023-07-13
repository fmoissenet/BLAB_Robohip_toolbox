% ROBOHIP
% Define Robot stylus tip location in flange coordinate system

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
Folder.data    = 'C:\Users\moissene\OneDrive - unige.ch\2022 - ROBOHIP\Développement\Biomécanique\BLAB_Robohip_toolbox\data\';
Folder.dep     = [Folder.toolbox,'dependencies\'];
addpath(Folder.toolbox);
addpath(genpath(Folder.dep));
cd(Folder.data);

% Load data
c3dFile = 'jointsTestFull.c3d';
[Marker,Event,firstFrame,nMarker,fMarker,markerNames] = LoadC3DFile(c3dFile);
n = size(Marker.KUKA_base_01,3);

% Set base coordinate system
Xb = Marker.KUKA_base_01-Marker.KUKA_base_02;
Xb = Xb./sqrt(sum(Xb.^2,1));
Yb = Marker.KUKA_base_04-Marker.KUKA_base_01;
Yb = Yb./sqrt(sum(Yb.^2,1));
Zb = cross(Xb,Yb);
Zb = Zb./sqrt(sum(Zb.^2,1));
Ob = Marker.KUKA_base_05+(-154.5*Xb)+(110*Yb)+(-5*Zb);
Tb = [Xb Yb Zb Ob; repmat([0 0 0 1],[1 1 n])];

% Set flange coordinate system
Of = (Marker.KUKA_flange_01+Marker.KUKA_flange_02)/2;
Xf = Marker.KUKA_flange_01-Marker.KUKA_flange_02;
Xf = Xf./sqrt(sum(Xf.^2,1));
Yf = Marker.KUKA_flange_03-Of;
Yf = Yf./sqrt(sum(Yf.^2,1));
Zf = cross(Xf,Yf);
Zf = Zf./sqrt(sum(Zf.^2,1));
Of = Of+(-5*Zf);
Tf = [Xf Yf Zf Of; repmat([0 0 0 1],[1 1 n])];

% Determine functional joint axes
for ijoint = 1:7
    range                    = fix(Event.start(ijoint)*fMarker)+1:fix(Event.start(ijoint+1)*fMarker)+1;
    [a,asi,asj,rA,rAsi,rAsj] = SARA_array3(Tb(:,:,range),Tf(:,:,range));
    vaxis(:,ijoint)          = mean(a,3);
    paxis(:,ijoint)          = mean(rA,3);
    clear range a asi asj rA rAsi rAsj;
end

% Compute plane angles between geometrical and functional axis
% /vertical axes Zb (axis 0), axis 1, axis 3, axis 5, axis 7
for ijoint = [1 3 5 7]
    if rad2deg(atan2(norm(cross(mean(Zb,3),vaxis(:,ijoint))), dot(mean(Zb,3),vaxis(:,ijoint)))) > 150
        vaxis(:,ijoint) = -vaxis(:,ijoint);
        angle(ijoint) = rad2deg(atan2(norm(cross(mean(Zb,3),vaxis(:,ijoint))), dot(mean(Zb,3),vaxis(:,ijoint))));
    else
        angle(ijoint) = rad2deg(atan2(norm(cross(mean(Zb,3),vaxis(:,ijoint))), dot(mean(Zb,3),vaxis(:,ijoint))));
    end
end
% /horizontal axes Yb (axis 0), axis 2, axis 4, axis 6
for ijoint = [2 4 6]
    if rad2deg(atan2(norm(cross(mean(Yb,3),vaxis(:,ijoint))), dot(mean(Yb,3),vaxis(:,ijoint)))) > 150
        vaxis(:,ijoint) = -vaxis(:,ijoint);
        angle(ijoint) = rad2deg(atan2(norm(cross(mean(Yb,3),vaxis(:,ijoint))), dot(mean(Yb,3),vaxis(:,ijoint))));
    else
        angle(ijoint) = rad2deg(atan2(norm(cross(mean(Yb,3),vaxis(:,ijoint))), dot(mean(Yb,3),vaxis(:,ijoint))));
    end
end

% Set points along axes of rotations
P1 = [paxis(1,2)+100*vaxis(1,2); paxis(2,2)+100*vaxis(2,2); paxis(3,2)+100*vaxis(3,2)];
P2 = [paxis(1,2)-100*vaxis(1,2); paxis(2,2)-100*vaxis(2,2); paxis(3,2)-100*vaxis(3,2)];
P3 = [paxis(1,4)+100*vaxis(1,4); paxis(2,4)+100*vaxis(2,4); paxis(3,4)+100*vaxis(3,4)];
P4 = [paxis(1,4)-100*vaxis(1,4); paxis(2,4)-100*vaxis(2,4); paxis(3,4)-100*vaxis(3,4)];
P5 = [paxis(1,6)+100*vaxis(1,6); paxis(2,6)+100*vaxis(2,6); paxis(3,6)+100*vaxis(3,6)];
P6 = [paxis(1,6)-100*vaxis(1,6); paxis(2,6)-100*vaxis(2,6); paxis(3,6)-100*vaxis(3,6)];
P7 = [mean(Ob(1,1,:),3)+0*mean(Zb(1,1,:),3); mean(Ob(2,1,:),3)+0*mean(Zb(2,1,:),3); mean(Ob(3,1,:),3)+0*mean(Zb(3,1,:),3)];
P8 = [mean(Ob(1,1,:),3)+1500*mean(Zb(1,1,:),3); mean(Ob(2,1,:),3)+1500*mean(Zb(2,1,:),3); mean(Ob(3,1,:),3)+1500*mean(Zb(3,1,:),3)];

% Set intersection points
% /Axis 2
L          = P1-P7;
M          = P1-P2;
N          = P7-P8;
A          = [M N];
T          = pinv(A)*L;
caxis(:,2) = P1-T(1)*(P1-P2);
clear L M N A T;
% /Axis 4
L          = P3-P7;
M          = P3-P4;
N          = P7-P8;
A          = [M N];
T          = pinv(A)*L;
caxis(:,4) = P3-T(1)*(P3-P4);
clear L M N A T;
% /Axis 6
L          = P5-P7;
M          = P5-P6;
N          = P7-P8;
A          = [M N];
T          = pinv(A)*L;
caxis(:,6) = P5-T(1)*(P5-P6);
clear L M N A T;

% Compute distance between horizontal axes (d parameter in Denavit
% Hartenberg notation)
d(1) = sqrt(sum((mean(Ob,3)-caxis(:,2)).^2));
d(2) = sqrt(sum((caxis(:,2)-caxis(:,4)).^2));
d(3) = sqrt(sum((caxis(:,4)-caxis(:,6)).^2));

% Plot
figure; hold on; axis equal;
plot3(mean(Marker.KUKA_base_01(1,1,:),3),mean(Marker.KUKA_base_01(2,1,:),3),mean(Marker.KUKA_base_01(3,1,:),3),'Marker','.','MarkerSize',10,'Color','black');
plot3(mean(Marker.KUKA_base_02(1,1,:),3),mean(Marker.KUKA_base_02(2,1,:),3),mean(Marker.KUKA_base_02(3,1,:),3),'Marker','.','MarkerSize',10,'Color','red');
plot3(mean(Marker.KUKA_base_03(1,1,:),3),mean(Marker.KUKA_base_03(2,1,:),3),mean(Marker.KUKA_base_03(3,1,:),3),'Marker','.','MarkerSize',10,'Color','black');
plot3(mean(Marker.KUKA_base_04(1,1,:),3),mean(Marker.KUKA_base_04(2,1,:),3),mean(Marker.KUKA_base_04(3,1,:),3),'Marker','.','MarkerSize',10,'Color','black');
quiver3(mean(Ob(1,1,:),3),mean(Ob(2,1,:),3),mean(Ob(3,1,:),3),mean(Xb(1,1,:),3),mean(Xb(2,1,:),3),mean(Xb(3,1,:),3),100,'red');
quiver3(mean(Ob(1,1,:),3),mean(Ob(2,1,:),3),mean(Ob(3,1,:),3),mean(Yb(1,1,:),3),mean(Yb(2,1,:),3),mean(Yb(3,1,:),3),100,'green');
quiver3(mean(Ob(1,1,:),3),mean(Ob(2,1,:),3),mean(Ob(3,1,:),3),mean(Zb(1,1,:),3),mean(Zb(2,1,:),3),mean(Zb(3,1,:),3),100,'blue');
line([P1(1) P2(1)],[P1(2) P2(2)],[P1(3) P2(3)],'Linestyle','--','Color','black');
line([P3(1) P4(1)],[P3(2) P4(2)],[P3(3) P4(3)],'Linestyle','--','Color','black');
line([P5(1) P6(1)],[P5(2) P6(2)],[P5(3) P6(3)],'Linestyle','--','Color','black');
line([P7(1) P8(1)],[P7(2) P8(2)],[P7(3) P8(3)],'Linestyle','--','Color','black');
plot3(caxis(1,2),caxis(2,2),caxis(3,2),'Marker','+','Color','red');
plot3(caxis(1,4),caxis(2,4),caxis(3,4),'Marker','+','Color','red');
plot3(caxis(1,6),caxis(2,6),caxis(3,6),'Marker','+','Color','red');

% c3dFile = 'grid_points.c3d';
% [Marker,Event,firstFrame,nMarker,fMarker,markerNames] = LoadC3DFile(c3dFile);
% 
% pointList = {'STY2_6_static' 'STY2_6_static_2' 'STY2_6_static_3' 'STY2_6_static_6' 'STY2_6_static_7'};
% eventList = {'grid_point01' 'grid_point02' 'grid_point03' 'grid_point06' 'grid_point07'};
% 
% for ipoint = 1:size(pointList,2)
%     error_point_X(ipoint)  = Marker.(pointList{ipoint})(1,:,fix(Event.(eventList{ipoint})*fMarker))-Marker.KUKA_flange_13(1,:,fix(Event.(eventList{ipoint})*fMarker));
%     error_point_Y(ipoint)  = Marker.(pointList{ipoint})(2,:,fix(Event.(eventList{ipoint})*fMarker))-Marker.KUKA_flange_13(2,:,fix(Event.(eventList{ipoint})*fMarker));
%     error_point_Z(ipoint)  = Marker.(pointList{ipoint})(3,:,fix(Event.(eventList{ipoint})*fMarker))-Marker.KUKA_flange_13(3,:,fix(Event.(eventList{ipoint})*fMarker));
%     error_point_3D(ipoint) = sqrt(sum((Marker.(pointList{ipoint})(:,:,fix(Event.(eventList{ipoint})*fMarker))-Marker.KUKA_flange_13(:,:,fix(Event.(eventList{ipoint})*fMarker))).^2,1));
% end

% c3dFile = 'Measurement6.c3d';
% Marker  = LoadC3DFile(c3dFile);
% n       = size(Marker.KUKA_base_01,3);
% 
% % Plate coordinate system
% Op = (Marker.PLATE_04+Marker.PLATE_08)/2;
% Xp = Marker.PLATE_01-Marker.PLATE_03;
% Xp = Xp./sqrt(sum(Xp.^2,1));
% Yp = Marker.PLATE_05-Marker.PLATE_03;
% Yp = Yp./sqrt(sum(Yp.^2,1));
% Zp = cross(Xp,Yp);
% Zp = Zp./sqrt(sum(Zp.^2,1));
% Yp = cross(Zp,Xp);
% Yp = Yp./sqrt(sum(Yp.^2,1));
% Tp = [Xp Yp Zp Op; repmat([0 0 0 1],[1 1 n])];
% 
% % Flange coordinate system
% Of = (Marker.KUKA_flange_01+Marker.KUKA_flange_02)/2;
% Xf = Marker.KUKA_flange_01-Marker.KUKA_flange_02;
% Xf = Xf./sqrt(sum(Xf.^2,1));
% Yf = Marker.KUKA_flange_03-Of;
% Yf = Yf./sqrt(sum(Yf.^2,1));
% Zf = cross(Xf,Yf);
% Zf = Zf./sqrt(sum(Zf.^2,1));
% Tf = [Xf Yf Zf Of; repmat([0 0 0 1],[1 1 n])];
% 
% figure; hold on; axis equal;
% plot3(Marker.KUKA_flange_01(1,1,1),Marker.KUKA_flange_01(2,1,1),Marker.KUKA_flange_01(3,1,1),'Marker','.','MarkerSize',10,'Color','black');
% plot3(Marker.KUKA_flange_02(1,1,1),Marker.KUKA_flange_02(2,1,1),Marker.KUKA_flange_02(3,1,1),'Marker','.','MarkerSize',10,'Color','red');
% plot3(Marker.KUKA_flange_03(1,1,1),Marker.KUKA_flange_03(2,1,1),Marker.KUKA_flange_03(3,1,1),'Marker','.','MarkerSize',10,'Color','black');
% quiver3(Of(1),Of(2),Of(3),Xf(1),Xf(2),Xf(3),100,'red');
% quiver3(Of(1),Of(2),Of(3),Yf(1),Yf(2),Yf(3),100,'green');
% quiver3(Of(1),Of(2),Of(3),Zf(1),Zf(2),Zf(3),100,'blue');
% 
% [rC,rCsi,rCsj,Residual] = SCoRE_array3(Tf,Tp);
% plot3(rC(1,1,1),rC(2,1,1),rC(3,1,1),'Marker','.','MarkerSize',10,'Color','blue');