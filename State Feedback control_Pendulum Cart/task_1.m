clc
clear 
close all

%% parameters
mc = 1.5;
mp = 0.5; 
g = 9.81;
L = 1; 
d1 = 0.01;
d2 = 0.01; %damping of the cart displacement and damping in the joint

%% Matrices

A = [0       0             1           0;
    0       0              0           1;
    0       (g*mp)/(mc)    -d1/(mc)   -d2/(mc); 
    0 (g*(mc+mp))/(L*mc)   -d1/(L*mc) -(d2*mc+d2*mp)/(L^2*mc*mp)];
B=[0;0;1/mc;1/(L*mc)];

%C=[0;1;0;0]; %q_2 as output
C=[1;0;0;0]; %q_1 as output

D=0;

%% building the system
sys = ss(A,B,C',D);
%eig(A) %to check system stability
% sc=ctrb(sys)
% so=obsv(sys)
% rank(sc)
% rank (so) %system is not observable with q_2 as output
%rlocus(sys)
x0=[0;5*pi/180;0;0]; %0=position,5 deg= angle , 0= velocity, 0=angular velocity

%% SF Controller
d_poles=[-3;-3;-3;-3]*0.5; %shifting all poles to -3
K=acker(A,B,d_poles)
Q=10*eye(4);
R=0.1;
K_lqr=lqr(A,B,Q,R);

%% Discrete Time model
Ts=0.1;
sys_d=c2d(sys,Ts);
Ad=sys_d.a;
Bd=sys_d.b;
Cd=sys_d.c;
Dd=sys_d.d;

d_poles_D=[0.3;0.3;0.3;0.3]*1;
K_d=acker(Ad,Bd,d_poles_D)

 %% Observer
d_poles_D=[0.3;0.3;0.3;0.3]*0.1;
ob=acker(Ad',Cd',d_poles_D)







