% Joshua Silver and Jordyn Watkins
% ENGR 454, Milestone 9, Full Order Observer
% June 8, 2021

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
Lind = 0.001;   
Cap2 = 0.0001;   % output capacitance
R2 = 27;         % buck converter output resistance
Vin = 9;
Cap1 = 0.000680; % input capacitance
R1 = 0.1;        % power supply output resistance
DD = 0.5;        % large signal duty cycle
IL = 0.1         % large signal current

% State Space Matrices in the time domain
% y = [iL; Vc2; Vc1];
A = [0 -1/Lind DD/Lind; 1/Cap2*Lind -1/(R2*Cap1) 0; -DD/Cap1 0 -1/(R1*Cap1)]
B = [Vin/Lind; 0; -IL/Cap1]
C = [0 1 0]
D = zeros(size(C,1),size(B,2))

uncontrolledsys=ss(A,B,C,D);

% Simulate results for uncontrolled system
t=0:0.001:0.5;              % times to simulate, start:step:stop
U = zeros(size(t));       % input, discrete values for each time
initial_current = 0.1;    
initial_vout = 6;      
initial_vcap1 = 9;
figure
[Y,T,X] = lsim(uncontrolledsys,U,t,[initial_current;initial_vout;initial_vcap1]);  % doesn't plot when output arguments are desired
plot(T,Y);
title("Uncontrolled System")

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

%% Full Order Observer Design: Creating the Observer

%observer functions
L = acker(A',C',[-2;-2;-2])'

Aobs = [A-L*C]
Bobs = [B L]
Cobs = eye(3)
Dobs = zeros(size(Cobs,1),size(Bobs,2))

obs_sys = ss(Aobs, Bobs,Cobs,Dobs);
%% Full Order Observer Design: Applying State Feedback

%LQR used to place poles
Q = [100 0 0; 0 10 0; 0 0 10];     % Minimize equation X'*Q*X - therefore Q could be thought of as Vd (input disturbances) in this context -- bigger Q means X stays smaller make this big so mistakes in this variable don't matter too much
%R = [0.1 0; 0 0.1];   % R is 2x2 if measuring both states in X  -- minimize equation Y'*R*Y -- Therefore R could be thought of as Vn (output noise) in this context
R = 100;              % R is 1x1 if measuring only one state in X
disp("K gain matrix with LQR placed kalman observer poles:")
K = lqr(Aobs,Bobs,Q,R)
P = eig(Aobs-Bobs*K)
G = place(Aobs, Bobs, P)

figure
t=0:0.001:0.5;              % times to simulate, start:step:stop
U = zeros(size(t));       % input, discrete values for each time
initial_current = 0.1;    
initial_vout = 6;      
initial_vcap1 = 9;
figure
[Y,T,X] = lsim(obs_sys,U,t,[initial_current;initial_vout;initial_vcap1]);  % doesn't plot when output arguments are desired
plot(T,Y);
title("Observed System")

%figure
%plot(Tfb, Xfb(:,1))
%title('Closed Loop Output Voltage Response to Step Input')