clc; clear all; close all;
%define linear system parameters
A = [2, -2; 1, 0];
B = [1; 1];
n = size(A,1);%degrees of freedom
m = size(B,2);%control inputs
C=eye(2);
Q = 5*eye(n);
R = 2*eye(m);
k=lqr(A,B,Q,R);
%check controlability
if(rank(ctrb(A,B))==n)
    disp('system is controllable');
else
    error('system is not controllable');
end
xo = [50; 0];% initial condition
k=[0,0];

x1=[]; %trajectories list for first variable
x2=[]; %trajectories list for second variable
T=[];

%generate different trajectories for the simulation 
for i=0.1:0.1:1
    [t,x]=ode45(@(t,x)linear_ode(A,B,C,k+i,t,x),[0,20],xo)
%     plot(t,x);
%     ti=[ti;t];
%     T=[T;x';]
    pause(1);
end

%fit polynomial curve to the data
p=polyfit(ti,T,n)


