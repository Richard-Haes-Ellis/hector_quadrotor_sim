%EIF Loop

function [Xk_1,Omegak_1,Xik_1] = EIFloop(Omegak,Xik,d1,d2,d3,d4,d5,gk,G,R,Q,H,d_s)

Omegak_inv = inv(Omegak);
Q_inv = inv(Q);

%Prediccion

Xk = Omegak_inv*Xik;
Omegak_1_P = inv(G*Omegak_inv*G'+R);

xk = Xk(1);yk = Xk(2);zk = Xk(3);xk_ant = Xk(4);yk_ant = Xk(5);zk_ant = Xk(6);vxk = Xk(7);vyk = Xk(8);vzk = Xk(9);

gkt = double(subs(gk));
Xik_1_P = Omegak_1_P*gkt;
Xk_1_P = gkt;

%Actualizacion
xk = Xk_1_P(1);yk = Xk_1_P(2);zk = Xk_1_P(3);xk_ant = Xk_1_P(4);yk_ant = Xk_1_P(5);zk_ant = Xk_1_P(6);vxk = Xk_1_P(7);vyk = Xk_1_P(8);vzk = Xk_1_P(9);

Ht = double(subs(H)); %Importante actualizar Ht con los valores de Xk predichos

Omegak_1 = double(Omegak_1_P+Ht'*Q_inv*Ht);

Xik_1 = double(Xik_1_P + Ht'*Q_inv*([d1;d2;d3;d4;d5]-subs(d_s)+Ht*Xk_1_P));
Xk_1 = inv(Omegak_1)*Xik_1;
end