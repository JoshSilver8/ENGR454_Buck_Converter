% Joshua Silver and Jordyn Watkins
% ENGR 454, Milestone 9, Full Order Observer
% May 28, 2021

%% Initializations
clear all
close all
clear variables
clc
format compact
%% Full Order Observer Design: System Matrices Dynamics
% we picked poles arbitrarily in the negative plane that removed complex
% components and decreased our response time.

% Buck Converter Component Values on our specific board
L = 0.001;
C = 0.0001;
R = 27;
Vin = 9;

% State Space Matrices in the time domain
% y = [iL; Vc];
A = [0 -1/L; 1/C -1/(R*C)]
B = [Vin/L; 0]
C = [0 1]
D = zeros(size(C,1),size(B,2))

%% Full Order Observer: checking for controllability and observability
%Checking for controllability and observability
R = ctrb(A,B)

if rank(A) == rank(R)
    disp('System is controllable')
else
    disp('System is NOT controllable')
end

Q = obsv(A,C)

if rank(A) == rank(Q)
    disp('System is controllable')
else
    disp('System is NOT controllable')
end

sys=ss(A,B,C,D);

figure
[Y,T,X]= step(sys);
plot(T,X(:,1));

%% Building Full Order Observer
%link to observer design tutorial: https://ctms.engin.umich.edu/CTMS/index.php?example=Introduction&section=ControlStateSpace
%http://www.asee.org/documents/zones/zone1/2014/Professional/PDFs/36.pdf
%https://www.mathworks.com/matlabcentral/answers/453779-how-to-design-a-full-order-observer-in-matlab


% Using LQE pole placement to build Kalman Filter:
% Augment system with disturbances and noise
Vd = [0.1 0; 0 0.1];  % disturbance covariance 
Vn = 1;               % noise co
[K2,P,E] = lqe(A,Vd,C,Vd,Vn);  % design Kalman filter 

Akf = [A-K2*C]
Bkf = B
Ckf = eye(2)
Dkf = zeros(2,1)

%need a way to solve for K
sysFOO = ss(Akf,Bkf,Ckf,Dkf);

K1 = acker(A,B,[-2;-100])

figure
[Yfb,Tfb,Xfb] = step(sysFOO);
plot(Tfb,Xfb(:,1));
title("Observer Response to Step Input");
sizeObs = size(Xfb)
size = size(X)

%% Full Order Observer Design: Adding Feedback

Qobs = [100 0 ; 0 50];
R = 200;

Kobs = lqr(Akf,Bkf,Qobs,R);
Pobs = eig(Akf-Bkf*Kobs)
Gobs = place(Akf, Bkf, Pobs)

AobsClosed = Akf-Bkf*Gobs
Eclosed = eig(AobsClosed)

sysclosed = ss(AobsClosed, Bkf, Ckf, Dkf);

figure
[Yfb,Tfb,Xfb] = step(sysclosed);
plot(Tfb,Xfb(:,1));
title('Closed Loop Output Voltage Response to Step Input')

figure
plot(Tfb,Xfb(1:127,1)-X(1:127,1));

%% Full Order Observer Design: Adding Disturbances and Noise for Simulation
%Vd = 0.1*eye(2) % disturbance covariance
%Vn = 1 % noise covariance

%BF = [B Vd 0*B]
%DF = [0 0 0 Vn]
%sysC = ss(A,BF,C,DF);

%sysFull = ss(A, BF, eye(2),zeros(2,size(BF,2)))
%% Building Observer, Kalman Filter
%[L,P,E] = lqe(A,Vd,C,Vd,Vn) % design kalman filter
%Kf = lqr(A',C',Vd,Vn)';

%sysKF = ss(A-Kf*C, [B Kf], eye(2), 0*[B Kf]);

%% Full Order Observer: Simulating the observer
%dt = 0.1;
%t = dt:dt:50;

%uDist = randn(2,size(t,2));
%uNoise = randn(size(t));
%u=0*t;
%u(100:120) = 100; %impulse
%u(1500:1520) = -100; %impulse

%uAUG = [u; Vd*Vd*uDist;uNoise];

%[y,t] = lsim(sysC,uAUG,t);
%plot(t,y);
%[xtrue,t] = lsim(sysFull,uAUG,t);




%det(gram(sys,'o'))


