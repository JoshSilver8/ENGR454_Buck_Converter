% Joshua Silver and Jordyn Watkins
% ENGR 454, Milestone 10, Reduced Order Observer
% June 2, 2021

%% Initializations
clear all
close all
clear variables
clc
format compact

%% Reduced Order Observer: Designing System Matrices Dynamics
% the state space equations and matrices were developed through circuit
% analysis in a previous homework. The buck converter simulated is a RLC
% buck converter, having both an output capacitor and output resistor.

% Buck Converter Component Values on our specific board
Lin = 0.001;
Cap = 0.0001;
R = 27;
Vin = 9;

% State Space Matrices in the time domain
% Note: the first row is the state space equation for Vc, and the second
% row is the state space equation for iL, so we have the form: x = [Vc; iL]
A = [-1/(R*Cap) 1/Cap; -1/Lin 0]
B = [0; Vin/Lin]
C = [1 0]   % we want to measure the output voltage, Vc, only
D = zeros(size(C,1),size(B,2))

sys = ss(A,B,C,D);
%% Reduced Order Observer: checking for controllability and observability
% If the system is both controllable and observable, meaning that the 
% controllability matrix and the observability matrix are both full rank,
% then we can theoretically control the system.

% Check for controllability
% Is the system still controllable only measuring Vc?
R = ctrb(A,B)
rank(R)
if rank(A) == rank(R)  
    disp('System is controllable')
else
    disp('System is NOT controllable')
end

% Check for observability
% Is the system still observable only when measuring Vc?
Q = obsv(A,C)
rank(Q)
if rank(A) == rank(Q)
    disp('System is controllable')
else
    disp('System is NOT controllable')
end

%% Reduced Order Observer: building the components
% The general formulas corresponding to the reduced order observer are:
% F = A22 - LC1A12
% H = B2 - LC1B1
% Gdoublebar = (A21 - LC1A11 + FLC1)*inv(C1)

% We define our sub-matrices as:
A11 = A(1,1)
A12 = A(1,2:end)
A21 = A(2:end,1)
A22 = A(2:end,2:end)
B1 = B(1,1)
B2 = B(2:end,1)
C1 = C(1,1)

% Pole placement - poles are found from the characteristic equation
% The characteristic equation 
PolesF = [-1000]

% solve for L matrix using the place function
L = place(A22',A12'*C1,PolesF)

% with L, we can solve for F, H, and Gbar
F = A22 - L*C1*A12
H = B2 - L*C1*B1
Gdoublebar = (A21 - L*C1*A11)*inv(C1)

% zdot = Fz + Gbar*y + H*u
% Gbar = (A21 -  L*C1*A11)*inv(C1) + F*L
% combining the above 2 equations gives us the following:
% zdot = Fz + ((A21 -  L*C1*A11)*inv(C1) + F*L)*y + H*u

% the other two formulas describing the reduced order observer are:
% x1 = x1hat = inv(C1)*y
% x2hat = L*y + z

%edot = e1dot/e2dot
%x1 = Vc
%x2 = iL

%x2dot =  A21*x1
%e2dot = x2dot - x2dothat

Aobs = zeros(3) 
Aobs(1:2, 1:2) = A
Aobs(3,1:3) = [L*C1+Gdoublebar*C1 0 F]
Bobs = [0; Vin/Lin; H]
Cobs = eye(3)
Dobs = zeros(3,1)

sys2 = ss(Aobs,Bobs,Cobs,Dobs);
[Y,T,X] = step(sys2)
eig(A)

figure
plot(T,X(:,3)- X(:,2))
title('error: ilhat-il')

% sys = ss(A,B,C,D)
% [Yr,Tr,Xr] = step(sys)
% plot(T,X(:,1)- X(:,2))


%% Applying state feedback

isLQR = 1;

if isLQR == 1
    Qobs = [100 0 0; 0 50 0; 0 0 50];
    R = 200;
    Kobs = lqr(Aobs,Bobs,Qobs,R);
    Pobs = eig(Aobs-Bobs*Kobs);
    Gobs = place(Aobs, Bobs, Pobs);
    
    Qno = [100 0; 0 50];
    Kno = lqr(A,B,Qno,R);
    Pno = eig(A-B*Kno);
    Gno = place(A,B, Pno);

    G = [Gno(1), Gno(2), Gobs(3)];
    
else
    P = [-600, -2500, -5700];
    G = place(Aobs,Bobs,P);
    Gno = place(A,B,[-600,-2500]);
end

AobsClosed = Aobs-Bobs*G

Eclosed = eig(AobsClosed)

sysclosed = ss(AobsClosed, Bobs, Cobs, Dobs)

figure
[Yfb,Tfb,Xfb] = step(sysclosed)
plot(Tfb,Xfb(:,3))
figure
plot(Tfb, Xfb(:,1))
title('Closed Loop Output Voltage Response to Step Input')

