function x_dot=func(A,B,C,k,t,x)
u=-k*x;
x_dot=A*x+B*u;
end