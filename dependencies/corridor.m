function []=corridor(X,Ymean,Ystd,color)
% Trace le corridor moyenne +/- ecart type
% mean, std = column vector
% x column vector[0:end,end:-1:0]
% Pour le trace des corridors, permet de faire un aller retour sur le
% graphe
hold on;
x=[X(1):1:X(end) X(end):-1:X(1)]';
y=[Ymean+Ystd;Ymean(end:-1:1)-Ystd(end:-1:1)]; % [Aller;Retour]
A=fill(x,y,color,'LineStyle','none','FaceAlpha',0.2); % Trace le corridor +/- 1 SD
set(get(get(A,'Annotation'),'LegendInformation'),'IconDisplayStyle','off'); % Retire surface de la légende
plot(X,Ymean,color);
