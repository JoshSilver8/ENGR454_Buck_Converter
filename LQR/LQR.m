% Joshua Silver and Jordyn Watkins
% ENGR 454, Milestone 8, LQR
% May 20, 2021

%%Initializations
clear all
close all
clc
%% Placing Poles by LQR
% we picked poles arbitrarily in the negative plane that removed complex
% components and decreased our response time.

% Buck Converter Component Values on our specific board
L = 0.001;
C = 0.0001;
R = 27;
Vin = 9;

% State Space Matrices in the time domain
A = [0 -1/L; 1/C -1/(R*C)]
B = [Vin/L; 0]
C = [0 1]
D = 0;

% we analyze the original model without feedback first to see how we should
% adjust our pole placement

% state space model of original system
sys = ss(A,B,C,D);

% eigenvalues of unmodified system
E = eig(A)

% step response of unmodified system
step(sys)


% now we choose poles, il and vc, by LQR
Q = [50 0; 0 100];
R = 100;
K = lqr(A,B,Q,R);

% Creating our linear feedback controller, G, using matlab's place function
P = eig(A-B*K)
G = place(A,B,P)

% updating our A matrix to (A-BG)x, since we are now including feedback
Aclosed = A-B*G

% verifiying our eigenvalues were placed correctly in the new A matrix.
Eclosed = eig(Aclosed)

% creating our closed loop system
sysclosed = ss(Aclosed, B, C, D);

% checking the step response of our closed loop system with desired pole
% placements
figure
step(sysclosed)

% correcting settling value by modifying the DCgain on the controller
Gdc = dcgain(sysclosed);
Gr = 9/Gdc;

% creating the new system with gain and checking the step response of the
% system
scaledsystem = ss(Aclosed, B*Gr, C, D);
figure
step(scaledsystem)

stepinfo(scaledsystem)