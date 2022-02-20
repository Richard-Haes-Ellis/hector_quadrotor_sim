clc;clear;close all;

elipse = 1; %Movimiento rectilineo (0) o eliptico (1)
Cte_elipse = 0.31;
if(elipse ==1)
    velocidadx=0.05;velocidady=0.05;velocidadz=0.025;
else
    velocidadx=0.15;velocidady=0.15;velocidadz=0.15;
end
T = 1;
Xk = [1;2;1;0;1;0;0.05;0.05;0.05];
% Xk = [1;2;1;0;1;0;velocidadx;velocidady;velocidadz];
syms xk yk zk xk_ant yk_ant zk_ant vxk vyk vzk real

%Posiciones de las balizas

x1 = 8; y1 = -7; z1 = 6;

x2 = 8; y2 = 2; z2 = 0;

x3 = 16; y3 = 2; z3 = 3;

x4 = 8; y4 = 12; z4 = 6;

x5 = -2; y5 = 2; z5 = 3;

%Matriz G
G = [1 0 0 0 0 0 T 0 0;...
     0 1 0 0 0 0 0 T 0;...
     0 0 1 0 0 0 0 0 T;...
     1 0 0 0 0 0 0 0 0;...
     0 1 0 0 0 0 0 0 0;...
     0 0 1 0 0 0 0 0 0;...
     1/T 0 0 -1/T 0 0 0 0 0;...
     0 1/T 0 0 -1/T 0 0 0 0;...
     0 0 1/T 0 0 -1/T 0 0 0];

%Funcion g

gk = [  xk+T*vxk;
        yk+T*vyk;
        zk+T*vzk;
        xk;
        yk;
        zk;
        (xk-xk_ant)/T;
        (yk-yk_ant)/T;
        (zk-zk_ant)/T];

%Matriz H
d1_s = sqrt(((xk-x1)^2)+((yk-y1)^2)+((zk-z1)^2));
d2_s = sqrt(((xk-x2)^2)+((yk-y2)^2)+((zk-z2)^2));
d3_s = sqrt(((xk-x3)^2)+((yk-y3)^2)+((zk-z3)^2));
d4_s = sqrt(((xk-x4)^2)+((yk-y4)^2)+((zk-z4)^2));
d5_s = sqrt(((xk-x5)^2)+((yk-y5)^2)+((zk-z5)^2));
d_s = [d1_s;d2_s;d3_s;d4_s;d5_s];

H = simplify([diff(d1_s,xk) diff(d1_s,yk) diff(d1_s,zk) diff(d1_s,xk_ant) diff(d1_s,yk_ant) diff(d1_s,zk_ant) diff(d1_s,vxk) diff(d1_s,vyk) diff(d1_s,vzk);...
              diff(d2_s,xk) diff(d2_s,yk) diff(d2_s,zk) diff(d2_s,xk_ant) diff(d2_s,yk_ant) diff(d2_s,zk_ant) diff(d2_s,vxk) diff(d2_s,vyk) diff(d2_s,vzk);...
              diff(d3_s,xk) diff(d3_s,yk) diff(d3_s,zk) diff(d3_s,xk_ant) diff(d3_s,yk_ant) diff(d3_s,zk_ant) diff(d3_s,vxk) diff(d3_s,vyk) diff(d3_s,vzk);...
              diff(d4_s,xk) diff(d4_s,yk) diff(d4_s,zk) diff(d4_s,xk_ant) diff(d4_s,yk_ant) diff(d4_s,zk_ant) diff(d4_s,vxk) diff(d4_s,vyk) diff(d4_s,vzk);...
              diff(d5_s,xk) diff(d5_s,yk) diff(d5_s,zk) diff(d5_s,xk_ant) diff(d5_s,yk_ant) diff(d5_s,zk_ant) diff(d5_s,vxk) diff(d5_s,vyk) diff(d5_s,vzk)]);

% Matriz Q

desvTPd1 = 0.25;desvTPd2 = 0.25;desvTPd3 = 0.25;desvTPd4=0.25;desvTPd5=0.25;
Q = diag([desvTPd1^2 desvTPd2^2 desvTPd3^2 desvTPd4^2 desvTPd5^2]);

%Matriz R

varModelo = 10000;
R = diag([varModelo varModelo varModelo*2 1e-5 1e-5 1e-5 varModelo*0.1 varModelo*0.1 varModelo*0.1]);

%Declaracion inicial de matriz y vector de informacion
ValorOmega = 0.1;
Omegak = diag([ValorOmega ValorOmega ValorOmega ValorOmega*10 ValorOmega*10 ValorOmega*10 ValorOmega*0.1 ValorOmega*0.1 ValorOmega*0.1]);
Xik = Omegak*Xk;
