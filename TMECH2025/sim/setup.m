% Citation infomation:
% W. Ren, G. R. Duan, P. Li, and H. Kong, 
% "Set-Based Fault-Tolerant Control for Continuous-Time Nonlinear Systems: 
% A Fully Actuated System Approach," IEEE/ASME Transactions on Mechatronics, 2025.
% DOI: 10.1109/TMECH.2025.3565876
% -------------------------------------------------------------------------
% Paper IV.A Simulation: A Coarse-Fine Tracking System
% -------------------------------------------------------------------------
% Additional Toolbox Needed:  LMI Toolbox (if the default Control System Toolbox are uninstalled)
% Additional Solver Needed:   None
% -------------------------------------------------------------------------
% Version:              1.0
% Author:               Weijie Ren
% Contact:              weijie.ren@outlook.com
% Initial modified:     Sep. 15, 2024
% Last modified:        Dec. 01, 2024
% -------------------------------------------------------------------------

clc,clear

%% Parameters From FAS
M_1r = [0 0 1]';
M_2r = [0 0 1]';
M_E = blkdiag(M_1r,M_2r);

%% Controller Parameters
% Design subsystem 1
Z_1 = [1 1 1];
cPole_1 = [-1 -2 -3];
F_1 = diag(cPole_1);
V_1 = [Z_1; Z_1*F_1; Z_1*F_1^2];
A_1 = -Z_1*F_1^3/V_1;
% Design subsystem 2
Z_2 = [1 1 1];
cPole_2 = [-1 -2 -3];
F_2 = diag(cPole_2);
V_2 = [Z_2; Z_2*F_2; Z_2*F_2^2];
A_2 = -Z_2*F_2^3/V_2;
% Write control gains together
K = blkdiag(A_1,A_2);
% Actuator faults' upper bound
f_a_sup = [10; 10];
% Practical compensation term
epsilon = [0; 0];
% Calculate positive P in quadratic Lyapunov function V = x^T P x
setlmis([])
P_1 = lmivar(1,[3,1]);
P_2 = lmivar(1,[3,1]);
lmiterm([1 1 1 P_1],1,[0 1 0; 0 0 1; -A_1],'s')
lmiterm([1 1 1 P_1],-2*max(cPole_1),1)
lmiterm([-2 1 1 P_1],1,1)
lmiterm([3 1 1 P_2],1,[0 1 0; 0 0 1; -A_2],'s')
lmiterm([3 1 1 P_2],-2*max(cPole_2),1)
lmiterm([-4 1 1 P_2],1,1)
lmisys = getlmis;
[tmin,xfeas] = feasp(lmisys);
P_1 = dec2mat(lmisys,xfeas,P_1);
P_2 = dec2mat(lmisys,xfeas,P_2);
P = blkdiag(P_1,P_2);

%% Detector Parameters
Phi_A_1 = [0 1 0;
           0 0 1;
           -A_1];
Phi_A_2 = [0 1 0;
           0 0 1;
           -A_2];
Phi_E = blkdiag(Phi_A_1,Phi_A_2);
% Calculate detector gain
[L_1,L_2,L_3,L_4] = deal(zeros(3,3));

p_1 = [-5 -4 -6];
for i = 1:3
    L_1(i,i) = -p_1(i);
end
for i = 1:2
    for j = i+1
        L_1(i,j) = 1 - 1;
    end
end
for i = 1:2
    L_1(3,i) = -A_1(i);
end
L_1(3,3) = L_1(3,3) - A_1(3);

L_2(1,1) = -0;
L_2(1,2) = 0;
L_2(1,3) = -1;

L_2(2,1) = -1;
L_2(2,2) = -0;
L_2(2,3) = -1;

L_2(3,1) = -0;
L_2(3,2) = -2;
L_2(3,3) = -1;

p_2 = [-10 -12 -14];
for i = 1:3
    L_4(i,i) = -p_2(i);
end
for i = 1:2
    for j = i+1
        L_4(i,j) = 1 - 1;
    end
end
for i = 1:2
    L_4(3,i) = -A_2(i);
end
L_4(3,3) = L_4(3,3) - A_2(3);

L_3(1,1) = -0;

L_3(2,1) = -1;
L_3(2,2) = -0;
L_3(2,3) = -1;

L_3(3,2) = -2;
L_3(3,3) = -3;

L = [L_1 L_2; L_3 L_4];
disp('The error state matrix:')
Phi_E-L
disp('The eigenvalues of error state matrix:')
eig(Phi_E-L)

% Set disturbance coefficient matrix and bound info.
% Assume the disturbance exists in actuator such that B*(u + fa + w). The
% coefficient D in the paper is actually B. For now, we find the bound of B
% as follows:
% Since B=[c_1        0]
%         [  0  c_2/M_2]
% c_1 and c_2 are constant with values 1.
% M_2 = 2 + cos(theta_2). Thus, 1 <= M_2 <= 3 and 1/3 <= c_2/M_2 <= 1.
% Then, we have
Dp = diag([1 1]);
Dm = zeros(2,2);
w_u = [1; 1];
w_l = -w_u;
varepsilon_u = [0.1 0.1 0.1 0.1 0.1 0.1]';
varepsilon_l = -varepsilon_u;

%% Simulation settings
% INITIAL_SYSTEM_VALUE = 1*(2*rand(6,1)-1);     % Random initial values
INITIAL_SYSTEM_VALUE = [0.3946 0.4599 -0.7478 -0.3090 -0.9610 0.0815]';
INITIAL_UPPER_OBSERVER_VALUE = [1 1 30 1 1 30]';
INITIAL_LOWER_OBSERVER_VALUE = -INITIAL_UPPER_OBSERVER_VALUE;

run('coarse_fine_tracking_system')