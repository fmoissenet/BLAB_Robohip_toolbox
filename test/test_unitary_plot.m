clearvars -except R d; clc; close all;
addpath('C:\Users\moissene\OneDrive - unige.ch\2022 - ROBOHIP\Développement\Biomécanique\BLAB_Robohip_toolbox\test');
cd('C:\Users\moissene\OneDrive - unige.ch\2022 - ROBOHIP\Développement\Biomécanique\BLAB_Robohip_toolbox\test\data_flangeAsBase3');
clear test_unitary_axis;

r = [];
t = [];
for ijoint = 1:3%1:7
    fileRobot = dir(['*axis',num2str(ijoint),'-1.mat*']);
    load(fileRobot.name);
    fileCamera = dir(['*axis',num2str(ijoint),'-1-camera.mat*']);
    load(fileCamera.name);
%     for imarker = 1:9
%         test_unitary_axis(ijoint).marker(imarker).camera = motion_capture_data.markers(imarker,:,1:3)*1e3;
%         test_unitary_axis(ijoint).marker(imarker).camera = squeeze(test_unitary_axis(ijoint).marker(imarker).camera);
%     end
%     test_unitary_axis(ijoint).flange.marker = (test_unitary_axis(ijoint).marker(1).camera+...
%                                                test_unitary_axis(ijoint).marker(3).camera)/2;
    test_unitary_axis(ijoint).flange.camera = motion_capture_data.six_dofs(1,:,1:3)*1e3;
    test_unitary_axis(ijoint).flange.camera = squeeze(test_unitary_axis(ijoint).flange.camera);
    test_unitary_axis(ijoint).flange.robot  = records.values(:,9:11)*1e3;
%     test_unitary_axis(ijoint).flange.marker = interp1(1:length(test_unitary_axis(ijoint).flange.marker), ...
%                                                   test_unitary_axis(ijoint).flange.marker, ...
%                                                   linspace(1, ...
%                                                            length(test_unitary_axis(ijoint).flange.marker), ...
%                                                            length(test_unitary_axis(ijoint).flange.robot)) ...
%                                                   );
    test_unitary_axis(ijoint).flange.camera = interp1(1:length(test_unitary_axis(ijoint).flange.camera), ...
                                                      test_unitary_axis(ijoint).flange.camera, ...
                                                      linspace(1, ...
                                                               length(test_unitary_axis(ijoint).flange.camera), ...
                                                               length(test_unitary_axis(ijoint).flange.robot)) ...
                                                      );
    clear fileRobot fileCamera motion_capture_data records;

    axisLabels = {'X' 'Y' 'Z'};
    for iaxis = 1:3
        range = 35:3095;
        r0(:,iaxis) = interp1(1:length(test_unitary_axis(ijoint).flange.camera(range,iaxis)), ...
                              test_unitary_axis(ijoint).flange.camera(range,iaxis),...
                              linspace(1,length(test_unitary_axis(ijoint).flange.camera(range,iaxis)),length(test_unitary_axis(ijoint).flange.robot(:,iaxis))));
        rt(:,iaxis) = fillmissing(r0(:,iaxis),'makima');
        tt(:,iaxis) = test_unitary_axis(ijoint).flange.robot(:,iaxis);
    end
    r = [r; rt];
    t = [t; tt];
    for iaxis = 1:3
        figure;
        subplot(1,2,1); hold on;
        title(['Axis ',num2str(ijoint),' - ',axisLabels{iaxis}]);
        plot(r0(:,iaxis),'black');
        plot(t(:,iaxis),'blue');
        plot(r(:,iaxis),'red');
        legend({'camera','robot'});
        subplot(1,2,2); hold on;
        plot(r(:,iaxis)-t(:,iaxis));
    end    
%     for iaxis = 1:3
%         rmax(ijoint,iaxis) = r(rind+fix(length(r0(:,iaxis))*20/100),iaxis);
%         tmax(ijoint,iaxis) = t(tind+fix(length(r0(:,iaxis))*20/100),iaxis);
%     end
%     clear r0 r t rind tind;
end

% [R,d,rms] = soder(r,t);
% r2 = (R*r'+d)';
% for iaxis = 1:3
%     figure;
%     subplot(1,2,1); hold on;
%     title(['Axis ',num2str(ijoint),' - ',axisLabels{iaxis}]);
%     plot(r(:,iaxis),'black');
%     plot(t(:,iaxis),'blue');
%     plot(r2(:,iaxis),'red');
%     legend({'camera','robot'});
%     subplot(1,2,2); hold on;
%     plot(r2(:,iaxis)-t(:,iaxis));
% end

% [R,d,rms] = soder(rmax([2 4 6],:),tmax([2 4 6],:));
% 
% figure(); hold on; axis equal;
% plot3(rmax([2 4 6],1),rmax([2 4 6],2),rmax([2 4 6],3),'Marker','.','Markersize',15,'Color','red');
% plot3(tmax([2 4 6],1),tmax([2 4 6],2),tmax([2 4 6],3),'Marker','.','Markersize',15,'Color','blue');
% rmax2 = (R*rmax([2 4 6],:)' + d)';
% plot3(rmax2(:,1)',rmax2(:,2)',rmax2(:,3)','Marker','.','Markersize',15,'Color','green');