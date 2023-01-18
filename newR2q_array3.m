% Author       : F. Moissenet
%                Biomechanics Laboratory (B-LAB)
%                University of Geneva
%                https://www.unige.ch/medecine/chiru/fr/b-lab-tests-robotises-avances-de-dispositifs-chirurgicaux/
% License      : Creative Commons Attribution-NonCommercial 4.0 International License 
%                https://creativecommons.org/licenses/by-nc/4.0/legalcode
% Source code  : https://github.com/fmoissenet/BLAB_Robohip_toolbox
% Reference    : To be defined
% Date         : December 2022
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

function Q_flange = newR2q_array3(R_flange)

temp     = permute(rotMat2quatern(R_flange),[2,3,1]);
Q_flange = circshift(temp,-1); % Required circularly shift to respect quaternion convention
for i = 2:size(Q_flange,3) % Continuity test (Lee's method)
    if norm(qlog_array3(qprod_array3(...
        qinv_array3(Q_flange(:,:,i-1)), Q_flange(:,:,i)))) > pi/2
        Q_flange(:,:,i) = - Q_flange(:,:,i);
    end
end
Q_flange(4,1,:) = -Q_flange(4,1,:);
for i = 1:size(Q_flange,3) % Normalise the quaternion
    Q_flange(:,:,i) = Q_flange(:,:,i)./sqrt(Q_flange(1,1,i)^2+...
                                            Q_flange(2,1,i)^2+...
                                            Q_flange(3,1,i)^2+...
                                            Q_flange(4,1,i)^2);
end