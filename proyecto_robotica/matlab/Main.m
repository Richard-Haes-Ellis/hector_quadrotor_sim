%Main

run('DeclaracionInicial.m');

Numero_iteraciones = 300;

X_real = Xk(1:3);
Det_Omega = zeros(1,Numero_iteraciones);
t = zeros(1,Numero_iteraciones);
Det_Omega(1) = det(Omegak);
t(1) = 0;

RESULT_X = zeros(18,size(Det_Omega,2));
RESULT_X(:,1) = [[X_real;0;1;0;velocidadx;velocidady;velocidadz];Xk];

for i=1:1:(size(Det_Omega,2)-1)
    t(i+1) = t(i)+T;
    %Trayectoria real del robot (RECTILINEA CON PARADA TEMPORAL A MITAD DE
    %CAMINO).
    if(elipse==0)
        if(i>=60 && i<80)
            RESULT_X(4:9,1+i) = [X_real;0;0;0]; %Guardamos la posicion anterior
            X_real = X_real+[0;0;0]*T;
        else
            RESULT_X(4:9,1+i) = [X_real;velocidadx;velocidady;velocidadz]; %Guardamos la posicion anterior
            X_real = X_real+[velocidadx;velocidady;velocidadz]*T;
        end
    elseif(elipse==1)
%         if(i>=60 && i<80)
%             RESULT_X(4:9,1+i) = [X_real;0;0;0]; %Guardamos la posicion anterior
%             X_real = X_real+[0;0;0]*T;
%         else
            RESULT_X(4:9,1+i) = [X_real;0;0;0]; %Guardamos la posicion anterior   
            X_real = X_real+[Cte_elipse*sin(velocidadx*t(i+1));Cte_elipse*cos(velocidadx*t(i+1));velocidadz*T];
%         end
    end
    
    %Simulacion de balizas
    d1 = sqrt(((X_real(1)-x1)^2)+((X_real(2)-y1)^2)+((X_real(3)-z1)^2))+normrnd(0,desvTPd1);
    d2 = sqrt(((X_real(1)-x2)^2)+((X_real(2)-y2)^2)+((X_real(3)-z2)^2))+normrnd(0,desvTPd2);
    d3 = sqrt(((X_real(1)-x3)^2)+((X_real(2)-y3)^2)+((X_real(3)-z3)^2))+normrnd(0,desvTPd3);    
    d4 = sqrt(((X_real(1)-x4)^2)+((X_real(2)-y4)^2)+((X_real(3)-z4)^2))+normrnd(0,desvTPd4);
    d5 = sqrt(((X_real(1)-x5)^2)+((X_real(2)-y5)^2)+((X_real(3)-z5)^2))+normrnd(0,desvTPd5);
    
    %Llamada al filtro
    [Xk,Omegak,Xik] = EIFloop(Omegak,Xik,d1,d2,d3,d4,d5,gk,G,R,Q,H,d_s);
    Det_Omega(i+1) = det(Omegak);
    
    %Guardar resultados
    RESULT_X(1:3,1+i) = X_real; %Guardamos la posicion
    RESULT_X(10:18,1+i) = Xk;
end

% x_filtrada = lowpass(RESULT_X(10,:),0.053);
% y_filtrada = lowpass(RESULT_X(11,:),0.053);
% z_filtrada = lowpass(RESULT_X(12,:),0.053);

%Representacion en 3D
figure;
plot3(RESULT_X(1,:),RESULT_X(2,:),RESULT_X(3,:),'.-b','LineWidth',1.1);grid;title('Representación de la trayectoria 3D');
xlabel('X (m)');ylabel('Y (m)');zlabel('Z (m)');
hold on;
plot3(RESULT_X(10,:),RESULT_X(11,:),RESULT_X(12,:),'.-r','LineWidth',1.1);
hold off;
% hold on;
% plot3(x_filtrada,y_filtrada,z_filtrada,'.-k','LineWidth',1.1);
% hold off;
hold on;
plot3(x1,y1,z1,'k*');
plot3(x2,y2,z2,'g*');
plot3(x3,y3,z3,'c*');
plot3(x4,y4,z4,'m*');
plot3(x5,y5,z5,'b*');
legend('Trayectoria real','Trayectoria estimada','Baliza 1','Baliza 2','Baliza 3','Baliza 4','Baliza 5');
hold off;

%Representacion de cada coordenada
figure;
title('Representación de cada coordenada');
subplot(3,1,1);
plot(t,RESULT_X(10,:),'r');grid;
xlabel('Tiempo (s)');ylabel('X (m)');
hold on;
plot(t,RESULT_X(1,:),'b');
hold off;
subplot(3,1,2);
plot(t,RESULT_X(11,:),'r');grid;
xlabel('Tiempo (s)');ylabel('Y (m)');
hold on;
plot(t,RESULT_X(2,:),'b');
hold off;
subplot(3,1,3);
plot(t,RESULT_X(12,:),'r');grid;
xlabel('Tiempo (s)');ylabel('Z (m)');
hold on;
plot(t,RESULT_X(3,:),'b');
hold off;

%Representaciones de los errores
figure;
title('Errores en cada coordenada');
subplot(3,1,1);
plot(t,abs(RESULT_X(1,:)-RESULT_X(10,:)));grid;
xlabel('Tiempo (s)');ylabel('Error en X (m)');
subplot(3,1,2);
plot(t,abs(RESULT_X(2,:)-RESULT_X(11,:)));grid;
xlabel('Tiempo (s)');ylabel('Error en Y (m)');
subplot(3,1,3);
plot(t,abs(RESULT_X(3,:)-RESULT_X(12,:)));grid;
xlabel('Tiempo (s)');ylabel('Error en Z (m)');
% hold on;
% plot(t,abs(RESULT_X(3,:)-z_filtrada));
% hold off;

disp('Error en z medio:');
media_error = sum(abs(RESULT_X(3,:)-RESULT_X(12,:)))/size(RESULT_X,2);
disp(media_error);