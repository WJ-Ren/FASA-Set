% Citation infomation:
% W. Ren, G. R. Duan, P. Li, and H. Kong, 
% "Set-Based Fault-Tolerant Control for Continuous-Time Nonlinear Systems: 
% A Fully Actuated System Approach," IEEE/ASME Transactions on Mechatronics, 2025.
% DOI: 10.1109/TMECH.2025.3565876
% -------------------------------------------------------------------------
% Paper IV.A Simulation: A Coarse-Fine Tracking System
%   >>>>This script plots results used in the paper.<<<<
% -------------------------------------------------------------------------
% Additional Toolbox Needed:  Zonotope Toolbox (for plotting box)
% Additional Solver Needed:   None
% -------------------------------------------------------------------------
% Version:              1.0
% Author:               Weijie Ren
% Contact:              weijie.ren@outlook.com
% Initial modified:     Sep. 15, 2024
% Last modified:        Dec. 01, 2024
% -------------------------------------------------------------------------

close all
t = ox.time;

%% Gershgorin Circle and Pole
xlimit = [-20 1];
figure, hold on, axis equal
tPhi = Phi_E-L;

for i = 1:6
    xx(i)  = tPhi(i,i);
    yy = 0;
    rr(i) = sum(abs(tPhi(i,:))) - abs(tPhi(i,i));
end
rectangle('Position',[xx(1)-rr(1),yy-rr(1),2*rr(1),2*rr(1)],'Curvature',[1,1],'FaceColor',[0 0.4470 0.7410 0.1],'LineStyle','-','LineWidth',1)
rectangle('Position',[xx(2)-rr(2),yy-rr(2),2*rr(2),2*rr(2)],'Curvature',[1,1],'FaceColor',[0.8500 0.3250 0.0980 0.1],'LineStyle','--','LineWidth',1)
rectangle('Position',[xx(3)-rr(3),yy-rr(3),2*rr(3),2*rr(3)],'Curvature',[1,1],'FaceColor',[0.9290 0.6940 0.1250 0.1],'LineStyle',':','LineWidth',1)
rectangle('Position',[xx(6)-rr(6),yy-rr(6),2*rr(6),2*rr(6)],'Curvature',[1,1],'FaceColor',[0.6350 0.0780 0.1840 0.1],'LineStyle','--','LineWidth',1)
rectangle('Position',[xx(5)-rr(5),yy-rr(5),2*rr(5),2*rr(5)],'Curvature',[1,1],'FaceColor',[0.4660 0.6740 0.1880 0.1],'LineStyle','-','LineWidth',1)
rectangle('Position',[xx(4)-rr(4),yy-rr(4),2*rr(4),2*rr(4)],'Curvature',[1,1],'FaceColor',[0.4940 0.1840 0.5560 0.1],'LineStyle','-.','LineWidth',1)
detectorPole = eig(Phi_E-L);
for i = 1:6
    plot(xx(i),0,'k.','MarkerSize',10)
    plot(real(detectorPole(i)),imag(detectorPole(i)),'b*','MarkerSize',5)
end

plot([xlimit(1) xlimit(2)],[0 0],'k-','LineWidth',1)
plot([0 0],[10 -10],'k-','LineWidth',1)
h = legend('Center of Gershgorin circle','Real pole location');
set(h,'Interpreter','LaTeX')
xlim([xlimit(1) xlimit(2)])
xlabel('Re')
ylabel('Im')
grid on


%% State Response and Control Input
figure
subplot(211)
plot(t,ox.signals.values(1,:),'-','LineWidth',1), hold on
plot(t,ox.signals.values(2,:),'--','LineWidth',1)
plot(t,ox.signals.values(4,:),'-.','LineWidth',1)
plot(t,ox.signals.values(5,:),':','LineWidth',1)
h = legend('$\theta_1$','$\dot{\theta}_1$','$\theta_2$','$\dot{\theta}_2$');
set(h,'Interpreter','LaTeX')
ylabel('State value')
grid on
ylim([-2 0.7])
% title('State response of FTC')

subplot(212)
plot(t,u.signals.values(1,:),'-','LineWidth',1), hold on
plot(t,u.signals.values(2,:),':','LineWidth',1)
h = legend('$u_1$','$u_2$');
set(h,'Interpreter','LaTeX')
xlabel('t (s)')
ylabel('Control value')
grid on
% title('Control input')

%% Set estimation
figure
subplot(231)
fill([t' fliplr(t')],[xh_u.signals.values(1,:) fliplr(xh_l.signals.values(1,:))],[0.47 0.67 0.19], ...
     'FaceAlpha',0.2,'EdgeColor','None','DisplayName','Reachable region'), hold on
plot(t,x.signals.values(1,:),'LineWidth',1,'Color',[0 .45 .74])
% plot(t,xh_u.signals.values(1,:),'--','LineWidth',1,'Color',[.85 .33 .01])
% plot(t,xh_l.signals.values(1,:),'-.','LineWidth',1,'Color',[.93 .69 .13])
plot(t,xh_u.signals.values(1,:),'--','LineWidth',1,'Color',[0.47 0.67 0.19])
plot(t,xh_l.signals.values(1,:),'-.','LineWidth',1,'Color',[0.47 0.67 0.19])
h = legend('', ...
           '$\theta_1, \dot{\theta}_1, \ddot{\theta}_1, \theta_2, \dot{\theta}_2, \ddot{\theta}_2$', ...
           '$\bar{\theta}_1, \dot{\bar{\theta}}_1, \ddot{\bar{\theta}}_1, \bar{\theta}_2, \dot{\bar{\theta}}_2, \ddot{\bar{\theta}}_2$', ...
           '');
set(h,'Interpreter','LaTeX')
ylabel('$\theta_1$','Interpreter','latex')
xlim([0 50])
ylim([-1 1])
grid on

subplot(232)
hh(1) = fill([t' fliplr(t')],[xh_u.signals.values(2,:) fliplr(xh_l.signals.values(2,:))],[0.47 0.67 0.19], ...
             'FaceAlpha',0.2,'EdgeColor','None','DisplayName','Estimated region'); hold on
plot(t,x.signals.values(2,:),'LineWidth',1,'Color',[0 .45 .74])
% plot(t,xh_u.signals.values(2,:),'--','LineWidth',1,'Color',[.85 .33 .01])
% hh(2) = plot(t,xh_l.signals.values(2,:),'-.','LineWidth',1,'Color',[.93 .69 .13],...
%              'DisplayName','$\underline{\theta}_1, \dot{\underline{\theta}}_1, \ddot{\underline{\theta}}_1, \underline{\theta}_2, \dot{\underline{\theta}}_2, \ddot{\underline{\theta}}_2$');
plot(t,xh_u.signals.values(2,:),'--','LineWidth',1,'Color',[0.47 0.67 0.19])
hh(2) = plot(t,xh_l.signals.values(2,:),'-.','LineWidth',1,'Color',[0.47 0.67 0.19],...
             'DisplayName','$\underline{\theta}_1, \dot{\underline{\theta}}_1, \ddot{\underline{\theta}}_1, \underline{\theta}_2, \dot{\underline{\theta}}_2, \ddot{\underline{\theta}}_2$');
h = legend(hh([2 1]));
set(h,'Interpreter','LaTeX')
ylabel('$\dot{\theta}_1$','Interpreter','latex')
xlim([0 50])
ylim([-1 1])
grid on

subplot(233)
fill([t' fliplr(t')],[xh_u.signals.values(3,:) fliplr(xh_l.signals.values(3,:))],[0.47 0.67 0.19], ...
     'FaceAlpha',0.2,'EdgeColor','None'), hold on
plot(t,x.signals.values(3,:),'LineWidth',1,'Color',[0 .45 .74])
% plot(t,xh_u.signals.values(3,:),'--','LineWidth',1,'Color',[.85 .33 .01])
% plot(t,xh_l.signals.values(3,:),'-.','LineWidth',1,'Color',[.93 .69 .13])
plot(t,xh_u.signals.values(3,:),'--','LineWidth',1,'Color',[0.47 0.67 0.19])
plot(t,xh_l.signals.values(3,:),'-.','LineWidth',1,'Color',[0.47 0.67 0.19])
ylabel('$\ddot{\theta}_1$','Interpreter','latex')
xlim([0 50])
ylim([-2.7 1.7])
grid on

subplot(234)
fill([t' fliplr(t')],[xh_u.signals.values(4,:) fliplr(xh_l.signals.values(4,:))],[0.47 0.67 0.19], ...
     'FaceAlpha',0.2,'EdgeColor','None'), hold on
plot(t,x.signals.values(4,:),'LineWidth',1,'Color',[0 .45 .74])
% plot(t,xh_u.signals.values(4,:),'--','LineWidth',1,'Color',[.85 .33 .01])
% plot(t,xh_l.signals.values(4,:),'-.','LineWidth',1,'Color',[.93 .69 .13])
plot(t,xh_u.signals.values(4,:),'--','LineWidth',1,'Color',[0.47 0.67 0.19])
plot(t,xh_l.signals.values(4,:),'-.','LineWidth',1,'Color',[0.47 0.67 0.19])
xlabel('t (s)')
ylabel('$\theta_2$','Interpreter','latex')
xlim([0 50])
ylim([-0.7 0.2])
grid on

subplot(235)
fill([t' fliplr(t')],[xh_u.signals.values(5,:) fliplr(xh_l.signals.values(5,:))],[0.47 0.67 0.19], ...
     'FaceAlpha',0.2,'EdgeColor','None'), hold on
plot(t,x.signals.values(5,:),'LineWidth',1,'Color',[0 .45 .74])
% plot(t,xh_u.signals.values(5,:),'--','LineWidth',1,'Color',[.85 .33 .01])
% plot(t,xh_l.signals.values(5,:),'-.','LineWidth',1,'Color',[.93 .69 .13])
plot(t,xh_u.signals.values(5,:),'--','LineWidth',1,'Color',[0.47 0.67 0.19])
plot(t,xh_l.signals.values(5,:),'-.','LineWidth',1,'Color',[0.47 0.67 0.19])
xlabel('t (s)')
ylabel('$\dot{\theta}_2$','Interpreter','latex')
xlim([0 50])
ylim([-0.7 1])
grid on

subplot(236)
fill([t' fliplr(t')],[xh_u.signals.values(6,:) fliplr(xh_l.signals.values(6,:))],[0.47 0.67 0.19], ...
     'FaceAlpha',0.2,'EdgeColor','None'), hold on
plot(t,x.signals.values(6,:),'LineWidth',1,'Color',[0 .45 .74])
% plot(t,xh_u.signals.values(6,:),'--','LineWidth',1,'Color',[.85 .33 .01])
% plot(t,xh_l.signals.values(6,:),'-.','LineWidth',1,'Color',[.93 .69 .13])
plot(t,xh_u.signals.values(6,:),'--','LineWidth',1,'Color',[0.47 0.67 0.19])
plot(t,xh_l.signals.values(6,:),'-.','LineWidth',1,'Color',[0.47 0.67 0.19])
xlabel('t (s)')
ylabel('$\ddot{\theta}_2$','Interpreter','latex')
xlim([0 50])
ylim([-1.5 2])
grid on
% title('Reachable set estimation')

%% Plot fault signals and fault flag
figure
subplot(211)
plot(t,f_a.signals.values(1,:),'-','LineWidth',1), hold on
plot(t,f_a.signals.values(2,:),':','LineWidth',1)
h = legend('$f_{a,1}$','$f_{a,2}$');
set(h,'Interpreter','LaTeX')
ylabel('Fault value')
ylim([-0.5 11])
grid on

subplot(212)
plot(t,faultyFlag.signals.values,'-','LineWidth',1)
ylim([-0.1 1.2])
h = legend('$\mathtt{fd}(\bar{\sigma},\underline{\sigma})$');
set(h,'Interpreter','LaTeX')
xlabel('t (s)')
ylabel('Logic value')
yticks([0 1])
grid on
% title('Fault detection result')


%% Plot Box
c_x1 = (xh_u.signals.values(1,:) +  xh_l.signals.values(1,:))/2;
c_x2 = (xh_u.signals.values(2,:) +  xh_l.signals.values(2,:))/2;
c_x3 = (xh_u.signals.values(3,:) +  xh_l.signals.values(3,:))/2;
c_x4 = (xh_u.signals.values(4,:) +  xh_l.signals.values(4,:))/2;
c_x5 = (xh_u.signals.values(5,:) +  xh_l.signals.values(5,:))/2;
c_x6 = (xh_u.signals.values(6,:) +  xh_l.signals.values(6,:))/2;
G_x1 = (xh_u.signals.values(1,:) -  xh_l.signals.values(1,:))/2;
G_x2 = (xh_u.signals.values(2,:) -  xh_l.signals.values(2,:))/2;
G_x3 = (xh_u.signals.values(3,:) -  xh_l.signals.values(3,:))/2;
G_x4 = (xh_u.signals.values(4,:) -  xh_l.signals.values(4,:))/2;
G_x5 = (xh_u.signals.values(5,:) -  xh_l.signals.values(5,:))/2;
G_x6 = (xh_u.signals.values(6,:) -  xh_l.signals.values(6,:))/2;

ops_point.Color = 'b';
ops_point.Linestyle = 'None';
ops_point.Marker = '*';
ops_point.MarkerSize = 6;
ops_point.LineWidth = 1;

clear ops_zono_healthy ops_zono_faulty
ops_zono_healthy.color = 'g';
ops_zono_healthy.shade = 0.2;
ops_zono_healthy.edgecolor = 'g';
% ops_zono_healthy.linewidth = 0.1;

ops_zono_faulty.color = 'r';
ops_zono_faulty.shade = 0.2;
ops_zono_faulty.edgecolor = 'r';
% ops_zono_faulty.linewidth = 30;

for i = 2:length(t)
    if faultyFlag.signals.values(i-1) == 0 && faultyFlag.signals.values(i) == 1
        nn1 = i;
    elseif faultyFlag.signals.values(i-1) == 1 && faultyFlag.signals.values(i) == 0
        nn2 = i;
    end
end

figure
nn = nn1-20;
subplot(2,8,1)
plot3(x.signals.values(1,nn),x.signals.values(2,nn),x.signals.values(3,nn),ops_point), hold on
zono = zonotope([c_x1(nn); c_x2(nn); c_x3(nn)],diag([G_x1(nn) G_x2(nn) G_x3(nn)]));
plotZ(zono,ops_zono_healthy)
xlabel('$\theta_1$','Interpreter','latex')
ylabel('$\dot{\theta}_1$','Interpreter','latex')
zlabel('$\ddot{\theta}_1$','Interpreter','latex')
title(['$t=$' num2str(t(nn),'%.2f') 's'],'Interpreter','latex')
% h = legend('Actual value','Set estimation when $\mathtt{fd}(\bar{\sigma},\underline{\sigma})$=0');
% set(h,'Interpreter','LaTeX')
subplot(2,8,9)
plot3(x.signals.values(1,nn),x.signals.values(2,nn),x.signals.values(3,nn),ops_point), hold on
zono = zonotope([c_x1(nn); c_x2(nn); c_x3(nn)],diag([G_x1(nn) G_x2(nn) G_x3(nn)]));
plotZ(zono,ops_zono_healthy)
xlabel('$\theta_2$','Interpreter','latex')
ylabel('$\dot{\theta}_2$','Interpreter','latex')
zlabel('$\ddot{\theta}_2$','Interpreter','latex')

nn = nn1-10;
subplot(2,8,2)
plot3(x.signals.values(1,nn),x.signals.values(2,nn),x.signals.values(3,nn),ops_point), hold on
zono = zonotope([c_x1(nn); c_x2(nn); c_x3(nn)],diag([G_x1(nn) G_x2(nn) G_x3(nn)]));
plotZ(zono,ops_zono_healthy)
xlabel('$\theta_1$','Interpreter','latex')
ylabel('$\dot{\theta}_1$','Interpreter','latex')
zlabel('$\ddot{\theta}_1$','Interpreter','latex')
title(['$t=$' num2str(t(nn),'%.2f') 's'],'Interpreter','latex')
subplot(2,8,10)
plot3(x.signals.values(1,nn),x.signals.values(2,nn),x.signals.values(3,nn),ops_point), hold on
zono = zonotope([c_x1(nn); c_x2(nn); c_x3(nn)],diag([G_x1(nn) G_x2(nn) G_x3(nn)]));
plotZ(zono,ops_zono_healthy)
xlabel('$\theta_2$','Interpreter','latex')
ylabel('$\dot{\theta}_2$','Interpreter','latex')
zlabel('$\ddot{\theta}_2$','Interpreter','latex')

nn = nn1;
subplot(2,8,3)
plot3(x.signals.values(1,nn),x.signals.values(2,nn),x.signals.values(3,nn),ops_point), hold on
zono = zonotope([c_x1(nn); c_x2(nn); c_x3(nn)],diag([G_x1(nn) G_x2(nn) G_x3(nn)]));
plotZ(zono,ops_zono_faulty)
xlabel('$\theta_1$','Interpreter','latex')
ylabel('$\dot{\theta}_1$','Interpreter','latex')
zlabel('$\ddot{\theta}_1$','Interpreter','latex')
title(['$t=$' num2str(t(nn),'%.2f') 's'],'Interpreter','latex')
% h = legend('','Set estimation when $\mathtt{fd}(\bar{\sigma},\underline{\sigma})$=1');
% set(h,'Interpreter','LaTeX')
subplot(2,8,11)
plot3(x.signals.values(1,nn),x.signals.values(2,nn),x.signals.values(3,nn),ops_point), hold on
zono = zonotope([c_x1(nn); c_x2(nn); c_x3(nn)],diag([G_x1(nn) G_x2(nn) G_x3(nn)]));
plotZ(zono,ops_zono_faulty)
xlabel('$\theta_2$','Interpreter','latex')
ylabel('$\dot{\theta}_2$','Interpreter','latex')
zlabel('$\ddot{\theta}_2$','Interpreter','latex')

nn = nn1+10;
subplot(2,8,4)
plot3(x.signals.values(1,nn),x.signals.values(2,nn),x.signals.values(3,nn),ops_point), hold on
zono = zonotope([c_x1(nn); c_x2(nn); c_x3(nn)],diag([G_x1(nn) G_x2(nn) G_x3(nn)]));
plotZ(zono,ops_zono_faulty)
xlabel('$\theta_1$','Interpreter','latex')
ylabel('$\dot{\theta}_1$','Interpreter','latex')
zlabel('$\ddot{\theta}_1$','Interpreter','latex')
title(['$t=$' num2str(t(nn),'%.2f') 's'],'Interpreter','latex')
subplot(2,8,12)
plot3(x.signals.values(1,nn),x.signals.values(2,nn),x.signals.values(3,nn),ops_point), hold on
zono = zonotope([c_x1(nn); c_x2(nn); c_x3(nn)],diag([G_x1(nn) G_x2(nn) G_x3(nn)]));
plotZ(zono,ops_zono_faulty)
xlabel('$\theta_2$','Interpreter','latex')
ylabel('$\dot{\theta}_2$','Interpreter','latex')
zlabel('$\ddot{\theta}_2$','Interpreter','latex')


%

nn = nn2-100;
subplot(2,8,5)
plot3(x.signals.values(1,nn),x.signals.values(2,nn),x.signals.values(3,nn),ops_point), hold on
zono = zonotope([c_x1(nn); c_x2(nn); c_x3(nn)],diag([G_x1(nn) G_x2(nn) G_x3(nn)]));
plotZ(zono,ops_zono_faulty)
xlabel('$\theta_1$','Interpreter','latex')
ylabel('$\dot{\theta}_1$','Interpreter','latex')
zlabel('$\ddot{\theta}_1$','Interpreter','latex')
title(['$t=$' num2str(t(nn),'%.2f') 's'],'Interpreter','latex')
subplot(2,8,13)
plot3(x.signals.values(1,nn),x.signals.values(2,nn),x.signals.values(3,nn),ops_point), hold on
zono = zonotope([c_x1(nn); c_x2(nn); c_x3(nn)],diag([G_x1(nn) G_x2(nn) G_x3(nn)]));
plotZ(zono,ops_zono_faulty)
xlabel('$\theta_2$','Interpreter','latex')
ylabel('$\dot{\theta}_2$','Interpreter','latex')
zlabel('$\ddot{\theta}_2$','Interpreter','latex')

nn = nn2-60;
subplot(2,8,6)
plot3(x.signals.values(1,nn),x.signals.values(2,nn),x.signals.values(3,nn),ops_point), hold on
zono = zonotope([c_x1(nn); c_x2(nn); c_x3(nn)],diag([G_x1(nn) G_x2(nn) G_x3(nn)]));
plotZ(zono,ops_zono_faulty)
xlabel('$\theta_1$','Interpreter','latex')
ylabel('$\dot{\theta}_1$','Interpreter','latex')
zlabel('$\ddot{\theta}_1$','Interpreter','latex')
title(['$t=$' num2str(t(nn),'%.2f') 's'],'Interpreter','latex')
subplot(2,8,14)
plot3(x.signals.values(1,nn),x.signals.values(2,nn),x.signals.values(3,nn),ops_point), hold on
zono = zonotope([c_x1(nn); c_x2(nn); c_x3(nn)],diag([G_x1(nn) G_x2(nn) G_x3(nn)]));
plotZ(zono,ops_zono_faulty)
xlabel('$\theta_2$','Interpreter','latex')
ylabel('$\dot{\theta}_2$','Interpreter','latex')
zlabel('$\ddot{\theta}_2$','Interpreter','latex')

nn = nn2;
subplot(2,8,7)
plot3(x.signals.values(1,nn),x.signals.values(2,nn),x.signals.values(3,nn),ops_point), hold on
zono = zonotope([c_x1(nn); c_x2(nn); c_x3(nn)],diag([G_x1(nn) G_x2(nn) G_x3(nn)]));
plotZ(zono,ops_zono_healthy)
xlabel('$\theta_1$','Interpreter','latex')
ylabel('$\dot{\theta}_1$','Interpreter','latex')
zlabel('$\ddot{\theta}_1$','Interpreter','latex')
title(['$t=$' num2str(t(nn),'%.2f') 's'],'Interpreter','latex')
subplot(2,8,15)
plot3(x.signals.values(1,nn),x.signals.values(2,nn),x.signals.values(3,nn),ops_point), hold on
zono = zonotope([c_x1(nn); c_x2(nn); c_x3(nn)],diag([G_x1(nn) G_x2(nn) G_x3(nn)]));
plotZ(zono,ops_zono_healthy)
xlabel('$\theta_2$','Interpreter','latex')
ylabel('$\dot{\theta}_2$','Interpreter','latex')
zlabel('$\ddot{\theta}_2$','Interpreter','latex')

nn = nn2+10;
subplot(2,8,8)
plot3(x.signals.values(1,nn),x.signals.values(2,nn),x.signals.values(3,nn),ops_point), hold on
zono = zonotope([c_x1(nn); c_x2(nn); c_x3(nn)],diag([G_x1(nn) G_x2(nn) G_x3(nn)]));
plotZ(zono,ops_zono_healthy)
xlabel('$\theta_1$','Interpreter','latex')
ylabel('$\dot{\theta}_1$','Interpreter','latex')
zlabel('$\ddot{\theta}_1$','Interpreter','latex')
title(['$t=$' num2str(t(nn),'%.2f') 's'],'Interpreter','latex')
subplot(2,8,16)
plot3(x.signals.values(1,nn),x.signals.values(2,nn),x.signals.values(3,nn),ops_point), hold on
zono = zonotope([c_x1(nn); c_x2(nn); c_x3(nn)],diag([G_x1(nn) G_x2(nn) G_x3(nn)]));
plotZ(zono,ops_zono_healthy)
xlabel('$\theta_2$','Interpreter','latex')
ylabel('$\dot{\theta}_2$','Interpreter','latex')
zlabel('$\ddot{\theta}_2$','Interpreter','latex')

