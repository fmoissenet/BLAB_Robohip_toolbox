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

function [Static,T_ics_hipref] = AddMotion(angles,axis,Static,T_ics_hipref)

% Set rotation matrix
if strcmp(axis,'e1')
    Tmotion = [ones(1,1,length(angles))       zeros(1,1,length(angles))      zeros(1,1,length(angles))      zeros(1,1,length(angles)); ...
               zeros(1,1,length(angles))      permute( cosd(angles),[1,3,2]) permute(-sind(angles),[1,3,2]) zeros(1,1,length(angles)); ...
               zeros(1,1,length(angles))      permute( sind(angles),[1,3,2]) permute( cosd(angles),[1,3,2]) zeros(1,1,length(angles)); ...
               zeros(1,1,length(angles))      zeros(1,1,length(angles))      zeros(1,1,length(angles))      ones(1,1,length(angles))];
elseif strcmp(axis,'e2')
    Tmotion = [permute( cosd(angles),[1,3,2]) zeros(1,1,length(angles))      permute( sind(angles),[1,3,2]) zeros(1,1,length(angles)); ...
               zeros(1,1,length(angles))      ones(1,1,length(angles))       zeros(1,1,length(angles))      zeros(1,1,length(angles)); ...
               permute(-sind(angles),[1,3,2]) zeros(1,1,length(angles))      permute( cosd(angles),[1,3,2]) zeros(1,1,length(angles)); ...
               zeros(1,1,length(angles))      zeros(1,1,length(angles))      zeros(1,1,length(angles))      ones(1,1,length(angles))];    
elseif strcmp(axis,'e3')
    Tmotion = [permute( cosd(angles),[1,3,2]) permute(-sind(angles),[1,3,2]) zeros(1,1,length(angles))      zeros(1,1,length(angles)); ...
               permute( sind(angles),[1,3,2]) permute( cosd(angles),[1,3,2]) zeros(1,1,length(angles))      zeros(1,1,length(angles)); ...
               zeros(1,1,length(angles))      zeros(1,1,length(angles))      ones(1,1,length(angles))       zeros(1,1,length(angles)); ...
               zeros(1,1,length(angles))      zeros(1,1,length(angles))      zeros(1,1,length(angles))      ones(1,1,length(angles))];    
end  

% Update landmark position
temp = Mprod_array3(repmat(T_ics_hipref,[1 1 length(angles)]),...
                    Mprod_array3(Tmotion,...
                                 repmat([Static.Femur.localLandmarks; ones(1,size(Static.Femur.landmarks,2),1)],[1 1 length(angles)])));
Static.Femur.landmarks = cat(3,Static.Femur.landmarks,temp(1:3,:,:));
clear temp;

% Update hip axes orientation
e1           = Vnorm_array3(Static.Pelvis.landmarks(:,1)-Static.Pelvis.landmarks(:,2)); % Flexion/extension axis
e3           = Vnorm_array3(Static.Femur.landmarks(:,3,end)-(Static.Femur.landmarks(:,1,end)+Static.Femur.landmarks(:,2,end))/2); % Internal/external axis
e2           = Vnorm_array3(cross(e3,e1));
T_ics_hipref = [e1 e2 e3 Static.Femur.landmarks(:,3,end); 0 0 0 1];

% Update femur landmarks in hip JCS
temp = Mprod_array3(Minv_array3(T_ics_hipref),[Static.Femur.landmarks(:,:,end); ones(1,size(Static.Femur.landmarks,2),1)]);
Static.Femur.localLandmarks = temp(1:3,:,:);