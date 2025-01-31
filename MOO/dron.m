clc, clear
close all

%% Parametre
Ix = 0.023;
Iy = 0.023;
Iz = 0.047;
m = 1.3;
g = 9.81;
IC = [0;
      0;
      0;
      0;
      0;
      0;
      0;
      0;
      0;
      0;
      0;
      0];

A = [0 0 0 0 0 0 1 0 0 0 0 0;  %1
     0 0 0 0 0 0 0 1 0 0 0 0;  %2
     0 0 0 0 0 0 0 0 1 0 0 0;  %3
     0 0 0 0 0 0 0 0 0 1 0 0;  %4
     0 0 0 0 0 0 0 0 0 0 1 0;  %5
     0 0 0 0 0 0 0 0 0 0 0 1;  %6
     0 0 0 0 g 0 0 0 0 0 0 0;
     0 0 0 -g 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0];

B = [0   0    0    0;
     0   0    0    0;
     0   0    0    0;
     0   0    0    0;
     0   0    0    0;
     0   0    0    0;
     0   0    0    0;
     0   0    0    0;
     1/m 0    0    0;
     0   1/Ix 0    0;
     0   0    1/Iy 0;
     0   0    0    1/Iz];

C = eye(12);
D = zeros(12,4);

%% controlability, obsv
rank(ctrb(A,B));
obs =  rank(obsv(A,C));

%% strisky
Aaa = A(1:6,1:6);
Aab = A(1:6,7:12);
Aba = A(7:12,1:6);
Abb = A(7:12,7:12);
Ba = B(1:6,:);
Bb = B(7:12,:);

J = [-8+4i -8-4i -20 -25 -30 -12 -10+2i -10-2i -18 -26 -6+i -6-i];
Je = [-16 -20 -35 -20 -18 -20]; %-10 -10 -62

K = place(A,B,J);  %4*12
Ka = K(1:4,1:6);
Kb = K(1:4,7:12);
Ke = place(Abb',Aab',Je)';

As = Abb-Ke*Aab;
Bs = As*Ke+Aba-Ke*Aaa;
Cs = [zeros(6,6); eye(6)];  %%%
Ds = [zeros(6,6)+1; Ke];     %%%
Fs = Bb - Ke*Ba;


At = As-Fs*Kb;
Bt = Bs-Fs*(Ka+Kb*Ke);
Ct = -Kb;
Dt = -(Ka+Kb*Ke);

num = [];
den = [];
% for i = 1:6
%     [num(i),den(i)] = ss2tf(At,Bt,-Ct,-Dt,i); %minus kvoli zapisu do danej ss2tf  -> ss2tf(At,Bt,-Ct,-Dt);
% end

[num1,den1] = ss2tf(At,Bt,-Ct,-Dt,1);
[num2,den2] = ss2tf(At,Bt,-Ct,-Dt,2);
[num3,den3] = ss2tf(At,Bt,-Ct,-Dt,3);
[num4,den4] = ss2tf(At,Bt,-Ct,-Dt,4);
[num5,den5] = ss2tf(At,Bt,-Ct,-Dt,5);
[num6,den6] = ss2tf(At,Bt,-Ct,-Dt,6);