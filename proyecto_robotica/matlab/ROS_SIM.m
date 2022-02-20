opengl('save','software')
clear all;
clc;
rosshutdown
rosinit

beacon_positions = [ 10.0, 10.0,  10.0;
				 	  2.0,  2.0,  0.0;
					 10.0,-10.0,  5.0;
					-10.0,-10.0,  10.0;
					-10.0, 10.0,  5.0];

RESULT_X = [];

results_sub = rossubscriber('/results','std_msgs/Float64MultiArray');

start_flight_pub = rospublisher("/start_flight","std_msgs/Empty");

msg = rosmessage(start_flight_pub);

send(start_flight_pub,msg);

for i=1:1:80
    results = receive(results_sub,2);
    RESULT_X = [RESULT_X; results.Data'];
end;

RESULT_X = RESULT_X';

%Representacion en 3D
figure;
plot3(RESULT_X(1,:),RESULT_X(2,:),RESULT_X(3,:),'.-b','LineWidth',1.1);grid;title('Representación de la trayectoria 3D');
xlabel('X (m)');ylabel('Y (m)');zlabel('Z (m)');
hold on;
plot3(RESULT_X(4,:),RESULT_X(5,:),RESULT_X(6,:),'.-r','LineWidth',1.1);
hold off;
% hold on;
% plot3(x_filtrada,y_filtrada,z_filtrada,'.-k','LineWidth',1.1);
% hold off;
hold on;
plot3(beacon_positions(1,1),beacon_positions(1,2),beacon_positions(1,3),'k*');
plot3(beacon_positions(2,1),beacon_positions(2,2),beacon_positions(2,3),'g*');
plot3(beacon_positions(3,1),beacon_positions(3,2),beacon_positions(3,3),'c*');
plot3(beacon_positions(4,1),beacon_positions(4,2),beacon_positions(4,3),'m*');
plot3(beacon_positions(5,1),beacon_positions(5,2),beacon_positions(5,3),'b*');
legend('Trayectoria real','Trayectoria estimada','Baliza 1','Baliza 2','Baliza 3','Baliza 4','Baliza 5');
hold off;

%Representacion de cada coordenada
figure;
title('Representación de cada coordenada');
subplot(3,1,1);
plot([1:1:length(RESULT_X(1,:))],RESULT_X(4,:),'r');grid;
xlabel('Tiempo (s)');ylabel('X (m)');
hold on;
plot([1:1:length(RESULT_X(1,:))],RESULT_X(1,:),'b');
hold off;
subplot(3,1,2);
plot([1:1:length(RESULT_X(1,:))],RESULT_X(5,:),'r');grid;
xlabel('Tiempo (s)');ylabel('Y (m)');
hold on;
plot([1:1:length(RESULT_X(1,:))],RESULT_X(2,:),'b');
hold off;
subplot(3,1,3);
plot([1:1:length(RESULT_X(1,:))],RESULT_X(6,:),'r');grid;
xlabel('Tiempo (s)');ylabel('Z (m)');
hold on;
plot([1:1:length(RESULT_X(1,:))],RESULT_X(3,:),'b');
hold off;

%Representaciones de los errores
figure;
title('Errores en cada coordenada');
subplot(3,1,1);
plot([1:1:length(RESULT_X(1,:))],abs(RESULT_X(1,:)-RESULT_X(4,:)));grid;
xlabel('Tiempo (s)');ylabel('Error en X (m)');
subplot(3,1,2);
plot([1:1:length(RESULT_X(1,:))],abs(RESULT_X(2,:)-RESULT_X(5,:)));grid;
xlabel('Tiempo (s)');ylabel('Error en Y (m)');
subplot(3,1,3);
plot([1:1:length(RESULT_X(1,:))],abs(RESULT_X(3,:)-RESULT_X(6,:)));grid;
xlabel('Tiempo (s)');ylabel('Error en Z (m)');
% hold on;
% plot(t,abs(RESULT_X(3,:)-z_filtrada));
% hold off;

disp('Error en z medio:');
media_error = sum(abs(RESULT_X(3,:)-RESULT_X(6,:)))/size(RESULT_X,2);
disp(media_error);


rosshutdown
