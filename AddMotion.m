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

function [T_ics_f,T_ics_fl,start,stop] = AddMotion(Subject,Folder,angles,rotationAxis,T_ics_f,T_ics_fl,T_f_fl,stop,order,motion)

% Set start and stop frames related to the generated motion
n = size(angles,2);
if size(T_ics_f,3) == 1 % Special case for motion set 1
    start = 2;
    stop  = n;
else
    start = stop+1;
    stop  = start+n-1;    
end

% Set rotation matrix
if strcmp(rotationAxis,'X')
    T_f2_f1   = [ones(1,1,length(angles))       zeros(1,1,length(angles))      zeros(1,1,length(angles))      zeros(1,1,length(angles)); ...
                 zeros(1,1,length(angles))      permute( cosd(angles),[1,3,2]) permute(-sind(angles),[1,3,2]) zeros(1,1,length(angles)); ...
                 zeros(1,1,length(angles))      permute( sind(angles),[1,3,2]) permute( cosd(angles),[1,3,2]) zeros(1,1,length(angles)); ...
                 zeros(1,1,length(angles))      zeros(1,1,length(angles))      zeros(1,1,length(angles))      ones(1,1,length(angles))];     
elseif strcmp(rotationAxis,'Y')
    T_f2_f1   = [permute( cosd(angles),[1,3,2]) zeros(1,1,length(angles))      permute( sind(angles),[1,3,2]) zeros(1,1,length(angles)); ...
                 zeros(1,1,length(angles))      ones(1,1,length(angles))       zeros(1,1,length(angles))      zeros(1,1,length(angles)); ...
                 permute(-sind(angles),[1,3,2]) zeros(1,1,length(angles))      permute( cosd(angles),[1,3,2]) zeros(1,1,length(angles)); ...
                 zeros(1,1,length(angles))      zeros(1,1,length(angles))      zeros(1,1,length(angles))      ones(1,1,length(angles))];  
elseif strcmp(rotationAxis,'Z')
    T_f2_f1   = [permute( cosd(angles),[1,3,2]) permute(-sind(angles),[1,3,2]) zeros(1,1,length(angles))      zeros(1,1,length(angles)); ...
                 permute( sind(angles),[1,3,2]) permute( cosd(angles),[1,3,2]) zeros(1,1,length(angles))      zeros(1,1,length(angles)); ...
                 zeros(1,1,length(angles))      zeros(1,1,length(angles))      ones(1,1,length(angles))       zeros(1,1,length(angles)); ...
                 zeros(1,1,length(angles))      zeros(1,1,length(angles))      zeros(1,1,length(angles))      ones(1,1,length(angles))]; 
end

% Store position/orientation of the femur in cameras coordinate system 
% for each frame of the motion
T_ics_f0 = T_ics_f(:,:,end); % Last generated bone pose
T_ics_f  = cat(3,T_ics_f,Mprod_array3(repmat(T_ics_f0,[1 1 size(T_f2_f1,3)]),Minv_array3(T_f2_f1)));

% Store position/orientation of the flange in cameras coordinate system 
% for each frame of the motion (apply a rigid transformation T_f_fl to the
% previously computed position/orientation of the femur)
T_ics_fl = cat(3,T_ics_fl,Mprod_array3(Mprod_array3(repmat(T_ics_f0,[1 1 size(T_f2_f1,3)]),Minv_array3(T_f2_f1)),repmat(T_f_fl,[1 1 size(Mprod_array3(repmat(T_ics_f0,[1 1 size(T_f2_f1,3)]),Minv_array3(T_f2_f1)),3)])));

% Generate the planning file for the KUKA robot
if order < 10
    ord = ['0',num2str(order)];
else
    ord = num2str(order);
end
O_flange = T_ics_fl(1:3,4,start:stop);
R_flange = [T_ics_fl(1:3,1,start:stop) -T_ics_fl(1:3,3,start:stop) T_ics_fl(1:3,2,start:stop)]; % Set correct flange axex from KUKA point of view
Q_flange = newR2q_array3(R_flange);
Name     = [ord,'_',motion,'_',Subject.id,'_',Subject.side,'.mat'];
cd(Folder.export); 
save(Name,'O_flange','Q_flange');