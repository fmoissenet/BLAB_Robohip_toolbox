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

function [Marker,Event,firstFrame,nMarker,fMarker,markerNames] = LoadC3DFile(c3dFile)

% Get real marker positions stored in C3D files
% -------------------------------------------------------------------------
btkFile     = btkReadAcquisition(c3dFile);
Marker      = btkGetMarkers(btkFile); % mm
Event       = btkGetEvents(btkFile);
markerNames = fieldnames(Marker); % Marker names in the marker trajectories
firstFrame  = btkGetFirstFrame(btkFile); % First frame number of the record
nMarker     = length(Marker.(markerNames{1})); % n frames stored in the marker trajectories
fMarker     = btkGetPointFrequency(btkFile); % Marker trajectories frequency

% Transform marker trajectory vectors as array3
% -------------------------------------------------------------------------
for imarker = 1:size(markerNames,1)
    Marker.(markerNames{imarker}) = permute(Marker.(markerNames{imarker}),[2,3,1]);
end

% Replace zeros by nan
% -------------------------------------------------------------------------
for imarker = 1:size(markerNames,1)
    for iframe = 1:nMarker
        if Marker.(markerNames{imarker})(:,:,iframe) == [0 0 0]
           Marker.(markerNames{imarker})(:,:,iframe) = nan(3,1,1);
        else
           Marker.(markerNames{imarker})(:,:,iframe) = Marker.(markerNames{imarker})(:,:,iframe);
        end
    end
end

% % Fill gaps
% % -------------------------------------------------------------------------
% tMarker = [];
% for imarker = 1:size(markerNames,1)
%     if ~isempty(Marker.(markerNames{imarker}))
%         tMarker = [tMarker permute(Marker.(markerNames{imarker}),[3,1,2])];
%     end
% end
% tMarker = PredictMissingMarkers(tMarker,'Algorithm',2);
% k = 0;
% for imarker = 1:size(markerNames,1)
%     if ~isempty(Marker.(markerNames{imarker}))
%         k = k+1;
%         Marker.(markerNames{imarker}) = permute(tMarker(:,(3*k)-2:3*k),[2,3,1]);
%     end
% end
% clear k tMarker;