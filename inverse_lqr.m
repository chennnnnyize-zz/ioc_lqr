%Inverse LQR Control
A = [2, -2; 1, 0];
B = [1; 1];
Q = 5*eye(2);
R = 2*eye(1);
n = size(A,1);%degrees of freedom
m = size(B,2);%control inputs

xo = [0.5; 0];

C=eye(2);D=[0;0];

states = {'x' 'x_dot'};
inputs = {'u'};
outputs = {'x'; 'x_dot'};
%system statespace
sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

%check controlability
co = ctrb(sys_ss);
controllability = rank(co);

if(controllability==n)
    disp('system is controllable');
else
    disp('system is not controllable');
end
syms a b c d;
P=[a b ; c d];
M=transpose(A)*P+P*A-(P*B*(R^-1)*transpose(B)*P)+Q;
sol=solve(M(:));
P=zeros([size(P),size(sol.a,1)]);
P(1,1,:)=sol.a;P(1,2,:)=sol.b;P(2,1,:)=sol.c;P(2,2,:)=sol.d;
d=[];
for i=1:size(sol.a,1)
    d=[d,det(P(:,:,i))];%find determinant
end
P=P(:,:,d>0); %consider only positive definite
disp(['there are ',num2str(sum(d>0)),' possible P'])
%gain matrix solving inverse lqr formula
K = -(R^-1)*transpose(B)*P(:,:,1);
% K=lqr(A,B,Q,R);

Ac = [(A-B*K)];
Bc = [B];
Cc = [C];
Dc = [D];

%system control
sys_cl = ss(Ac,Bc,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);

t = 0:0.01:5;
u =-1*ones(size(t));
[y,t,x]=lsim(sys_cl,u,t,xo);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','x1')
set(get(AX(2),'Ylabel'),'String','x2')
title('Step Response with LQR Control')



