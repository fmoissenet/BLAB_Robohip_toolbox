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

function Flange = ComputeFlangePosition(Flange,Static,Subject,Robot,Folder,Ord,Mvt,start,stop)

% Get flange marker trajectories
END01 = Static.Femur.landmarks(1:3,4,start:stop);
END02 = Static.Femur.landmarks(1:3,5,start:stop);
END03 = Static.Femur.landmarks(1:3,6,start:stop);
END04 = Static.Femur.landmarks(1:3,7,start:stop);
END05 = Static.Femur.landmarks(1:3,8,start:stop);
% Set flange coordinate system
Xfl = Vnorm_array3(END03-END01);
Yfl = Vnorm_array3(END02-END04);
Zfl = Vnorm_array3(cross(Xfl,Yfl));
Ofl = END05-Robot.ThicknessBotaPlate*1e-3/2*Zfl;
% Set flange trajectory and related quaternion (in m)
O_flange        = Ofl*1e-3;
R_flange        = [Xfl Yfl Zfl];
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
% Export results 
Flange.Ofl(:,:,start:stop) = Ofl;
Flange.Xfl(:,:,start:stop) = Xfl;
Flange.Yfl(:,:,start:stop) = Yfl;
Flange.Zfl(:,:,start:stop) = Zfl;
cd(Folder.export); 
Name      = [Ord,'_',Mvt,'_',Subject.side,'.mat'];
Namefull  = [Mvt,'_',Subject.side,'_','full.mat'];
save(Name,'O_flange','Q_flange');
save(Namefull);