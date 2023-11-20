% Author     :   F. Moissenet
%                Kinesiology Laboratory (K-LAB)
%                University of Geneva
%                https://www.unige.ch/medecine/kinesiology
% License    :   Creative Commons Attribution-NonCommercial 4.0 International License 
%                https://creativecommons.org/licenses/by-nc/4.0/legalcode
% Source code:   To be defined
% Reference  :   To be defined
% Date       :   April 2022
% -------------------------------------------------------------------------
% Description:   Stylus calibration
% -------------------------------------------------------------------------
% Dependencies : To be defined
% -------------------------------------------------------------------------
% This work is licensed under the Creative Commons Attribution - 
% NonCommercial 4.0 International License. To view a copy of this license, 
% visit http://creativecommons.org/licenses/by-nc/4.0/ or send a letter to 
% Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
% -------------------------------------------------------------------------

% function STY06 = CalibrateStylus(Folder)

% Load marker trajectories stored in C3D files
% cd(Folder.toolbox);
cd('C:\Users\moissene\OneDrive - unige.ch\2022 - ROBOHIP\Data\Tests\STYLUSa_20231019');
c3dFile = 'Measurement18.c3d';
btkFile = btkReadAcquisition(c3dFile);
Marker  = btkGetMarkers(btkFile);
nMarker = length(Marker.STYLUSa_01); % n frames stored in the marker trajectories
% Define stylus coordinate system
Os = Marker.STYLUSa_02;
Ys = (Marker.STYLUSa_02-Marker.STYLUSa_04);
Ys = Ys./sqrt(Ys(:,1).^2+Ys(:,2).^2+Ys(:,3).^2); 
Zs = (Marker.STYLUSa_01-Marker.STYLUSa_03);
Zs = Xs./sqrt(Xs(:,1).^2+Xs(:,2).^2+Xs(:,3).^2);
Xs = cross(Ys,Zs);
Zs = cross(Xs,Ys);
for i = 1:nMarker
    T_ics_s(i,1:4,1:4) = [[Xs(i,:)' Ys(i,:)' Zs(i,:)'] Os(i,:)'; 0 0 0 1]; % from s to ics
end
% Compute the averaged centre of rotation between T_ics_s and T_ics_ics
T_ics_ics      = repmat(permute([[1 0 0; 0 1 0; 0 0 1] [0 0 0]'; 0 0 0 1],[3,1,2]),[nMarker 1 1]); % from ics to ics
[rC,rCsi,rCsj] = SCoRE_array3(permute(T_ics_s,[2,3,1]),permute(T_ics_ics,[2,3,1]));
STY2_6         = rCsi % local position