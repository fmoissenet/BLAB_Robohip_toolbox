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

function [T_ics_f,T_ics_fl,start,stop] = AddMotion(T_ics_p,Specimen,Folder,angles,rotationAxis,T_ics_f,T_ics_fl,T_f_fl,stop,order,motion)

% Set start and stop frames related to the generated motion
n = size(angles,2);
if size(T_ics_f,3) == 1 % Special case for motion set 1
    start = 2;
    stop  = n;
else
    start = stop+1;
    stop  = start+n-1;    
end

% Define rotation axes
if contains(motion,'anatomicAlignement') % ICS axes are used in this case
    X_ics = [1 0 0];
    Y_ics = [0 1 0];
    Z_ics = [0 0 1];
else % JCS axes in all other cases
    Y_ics = T_ics_f(1:3,2,end);
    Y_ics = Y_ics/norm(Y_ics);
    Z_ics = T_ics_p(1:3,3,end);
    Z_ics = Z_ics/norm(Z_ics);
    X_ics = cross(Y_ics,Z_ics);
    X_ics = X_ics/norm(X_ics);
end

% Set 3D rotation matrix from axis and angle (https://en.wikipedia.org/wiki/Rotation_matrix)
if strcmp(rotationAxis,'X')
    ux = X_ics(1);
    uy = X_ics(2);
    uz = X_ics(3);
elseif strcmp(rotationAxis,'Y')
    ux = Y_ics(1);
    uy = Y_ics(2);
    uz = Y_ics(3);
elseif strcmp(rotationAxis,'Z')
    ux = Z_ics(1);
    uy = Z_ics(2);
    uz = Z_ics(3);
end
T = [permute(cosd(angles),[1,3,2])+ux^2*(1-permute(cosd(angles),[1,3,2]))     ux*uy*(1-permute(cosd(angles),[1,3,2]))-uz*permute(sind(angles),[1,3,2]) ux*uz*(1-permute(cosd(angles),[1,3,2]))+uy*permute(sind(angles),[1,3,2]) zeros(1,1,size(angles,2)); ...
     uy*ux*(1-permute(cosd(angles),[1,3,2]))+uz*permute(sind(angles),[1,3,2]) permute(cosd(angles),[1,3,2])+uy^2*(1-permute(cosd(angles),[1,3,2]))     uy*uz*(1-permute(cosd(angles),[1,3,2]))-ux*permute(sind(angles),[1,3,2]) zeros(1,1,size(angles,2)); ...
     uz*ux*(1-permute(cosd(angles),[1,3,2]))-uy*permute(sind(angles),[1,3,2]) uz*uy*(1-permute(cosd(angles),[1,3,2]))+ux*permute(sind(angles),[1,3,2]) permute(cosd(angles),[1,3,2])+uz^2*(1-permute(cosd(angles),[1,3,2]))     zeros(1,1,size(angles,2)); ...
     zeros(1,1,size(angles,2))                                                zeros(1,1,size(angles,2))                                                zeros(1,1,size(angles,2))                                                ones(1,1,size(angles,2))];

% Store position/orientation of the femur in cameras coordinate system 
% for each frame of the motion
T_ics_f0 = T_ics_f(:,:,end); % Last generated bone pose
if contains(motion,'anatomicAlignement')
    T_ics_f = cat(3,T_ics_f,Mprod_array3(repmat(T_ics_f0,[1 1 size(T,3)]),Minv_array3(T)));
else
    temp = Mprod_array3(T(1:3,1:3,:),repmat(T_ics_f0(1:3,1:3,:),[1 1 size(T,3)]));
    T_ics_f = cat(3,T_ics_f,[temp repmat(T_ics_f0(1:3,4,:),[1 1 size(T,3)]); repmat([0 0 0 1],[1 1 size(T,3)])]);
    clear temp;
end

% Store position/orientation of the flange in cameras coordinate system 
% for each frame of the motion (apply a rigid transformation T_f_fl to the
% previously computed position/orientation of the femur)
T_ics_fl = Mprod_array3(T_ics_f,repmat(T_f_fl,[1 1 size(T_ics_f,3)]));

% Generate the planning file for the KUKA robot
if order < 10
    ord = ['0',num2str(order)];
else
    ord = num2str(order);
end
O_flange = T_ics_fl(1:3,4,start:stop)*1e-3; % mm to m
R_flange = T_ics_fl(1:3,1:3,start:stop);
Q_flange = newR2q_array3(R_flange);
Name     = [ord,'_',motion,'_',Specimen.id,'_',Specimen.side,'.mat'];
cd(Folder.export); 
save(Name,'O_flange','Q_flange');