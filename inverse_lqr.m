% % Inverse LQR Control
clc;clear all; close all;
A = [2, -2; 1, 0];
B = [1; 1];
n = size(A,1);%degrees of freedom
m = size(B,2);%control inputs
Q = 5*eye(n);
R = 2*eye(m);
xo = [0.5; 0];
C=eye(2);
%check controlability
if(rank(ctrb(A,B))==n)
    disp('system is controllable');
else
    error('system is not controllable');
end
K=-lqr(A,B,Q,R); % let say we know this K before then we need to get that same Q and R
clear Q R;
% using yalmip package, and sedumi solver
Q=sdpvar(n,n,'full'); R=sdpvar(m,m,'full') ;P=sdpvar(n,n,'full') ;P1=sdpvar(n,n,'full') ;a=sdpvar(1,1,'full');
c1=[transpose(A)*P+P*A+(P*B*K)+Q==0];
c2=[transpose(A+B*K)*P+P*(A+B*K)+transpose(K)*R*K+Q==0];
c3=[transpose(B)*P+R*K==0];
c4=[transpose(A)*P1+P1*transpose(A)<Q];
c5=[eye(m+n)<=[[Q zeros(n,m)];[zeros(m,n) R]]<=a.*eye(m+n)];
constraints=[c1,c2,c3,c4,c5];
optimize(constraints,a^2)
Q=value(Q)
R=value(R)
P=value(P)
k=-lqr(A,B,Q,R) % output of inverse Q R 
K               %given input 