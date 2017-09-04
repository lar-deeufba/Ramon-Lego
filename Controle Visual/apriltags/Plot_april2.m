clear all; close all; clc

% Lê os arquivos
load('Dados.txt');
load('Dadosk.txt');
load('DadosZ.txt');

% Posicoes filtro kalman
Xk = Dadosk(:,1);
Yk = Dadosk(:,2);
Theta_k = Dadosk(:,9);
% Progressão do erro
Ex = Dados(:,4);
Ey = Dados(:,5);
Etheta = Dados(:,6);
% Dados sensor
rZ = DadosZ(:,1);
fiZ = DadosZ(:,2);
iZ = DadosZ(:,3);
mZ = DadosZ(:,4);
% Velocidades reais
W = Dadosk(:,5:6);
% Sinais de controle das rodas
U = Dadosk(:,7:8);
% Referencias
Ref_d = Dadosk(:,3);
Ref_e = Dadosk(:,4);
% Tamanho dos vetores
qk = length(Xk);
qm = length(W);
qz = length(rZ);
P1 = [Xk(1); Yk(1); Theta_k(1)];
% Estimação odométrica
for i=1:qm
    Xm(i,1) = P1(1);
    Ym(i,1) = P1(2);
    Theta_m(i,1) = P1(3);
    P1 = ModeloNL(P1, W(i,1), W(i,2), 0.6, 0.066, 0.0278);
end
%% Gera gráficos
Ref_d = (Ref_d./127)*17.3398;
Ref_e = (Ref_e./127)*17.3398;
% Gera gráficos da roda Direita
plot(W(:,1),'ro');
hold on;
plot(Ref_d,'bo')
stairs(W(:,1),'r');
stairs(Ref_d,'b');
title('Velocidade da Roda Direita');
xlabel('amostra'); ylabel('rad/s');
legend('Velocidade Medida','Referencia');
% figure
% % Gráficos dos sinais de controle aplicados
% plot(U(:,1),'bo');
% stairs(U(:,1),'b');
% title('Sinal de Controle da Roda Direita');

% Gera gráficos da roda Esquerda
figure
plot(W(:,2),'ro');
hold on;
plot(Ref_e,'bo');
stairs(W(:,2),'r');
stairs(Ref_e,'b');
title('Velocidade da Roda Esquerda');
xlabel('amostra'); ylabel('rad/s');
legend('Velocidade Medida','Referencia');
figure
% % Gráficos dos sinais de controle aplicados
% plot(t,U(:,2),'bo');
% stairs(t,U(:,2),'b');
% title('Sinal de Controle da Roda Esquerda')
% figure

% Plots para comparação
plot(Ref_d,'bo'); hold on;
plot(Ref_e,'ro'); 
stairs(Ref_d,'b'); 
stairs(Ref_e,'r'); 
title('Velocidades Desejadas');
xlabel('amostra'); ylabel('rad/s');
legend('Roda Direita','Roda Esquerda');
 
figure
plot(W(:,1),'bo'); hold on;
plot(W(:,2),'ro'); 
stairs(W(:,1),'b'); 
stairs(W(:,2),'r'); 
title('Velocidades Medidas');
xlabel('amostra'); ylabel('rad/s');
legend('Roda Direita','Roda Esquerda');
% 
figure
plot(U(:,1),'bo'); hold on;
plot(U(:,2),'ro'); 
stairs(U(:,1),'b'); 
stairs(U(:,2),'r'); 
title('Sinais de Controle');
xlabel('amostra'); ylabel('Valor em 8bits');
legend('Roda Direita','Roda Esquerda'); 
figure

% Gera graficos da posiçao 
h1 = plot(Yk(1,1),Xk(1,1),'go','MarkerSize',6);
hold on
h2 = plot(Yk(qk,1),Xk(qk,1),'mo','MarkerSize',6);
plot(Yk(1,1),Xk(1,1),'go','MarkerSize',6);
plot(Yk(1:qk-1),Xk(1:qk-1),'ro','MarkerSize',4);
h3 = plot(Yk(qk),Xk(qk),'ro','MarkerSize',6);
plot(Yk(qk),Xk(qk),'mo','MarkerSize',6);
plot(Yk,Xk,'r','LineWidth', 0.1);
plot(Ym(1,1),Xm(1,1),'g*','MarkerSize',6);
plot(Ym(1:qm,1),Xm(1:qm,1),'b*','MarkerSize',4);
h4 = plot(Ym(qm,1),Xm(qm,1),'b*','MarkerSize',6);
plot(Ym(qm,1),Xm(qm,1),'m*','MarkerSize',6);
plot(Ym,Xm,'b','LineWidth', 0.1);
%axis([-1 1 0 2.5]);
grid;
title('Posicao Estimada (m)')
xlabel('Y'); ylabel('X');
legend([h1, h2, h3, h4],{'Posicao Inicial','Posicao Final',...
                                      'Kalman','Odometria'});
figure

% Gráfico do ângulo
stairs(Theta_k(:,1)*180./pi,'r--');
hold on
stairs(Theta_m(:,1)*180./pi),'b--';
title('Ângulo estimado (graus)')
legend({'Kalman','Odometria'});
ylabel('\Theta'); xlabel('t');

figure
% Plota os Erros
h1 = stairs(Ex,'o-b');
hold on
stairs(Ey,'-or');
title('Progressão do Erro de Posição (m)')
legend({'X','Y'});
ylabel('(m)'); xlabel('t');

figure
stairs(Etheta*180/pi,'-ob')
title('Progressão do Erro do Angulo (graus)')
ylabel('(graus)'); xlabel('t');


% Graficos das posições
figure
stairs(Xk(:,1),'r--');
hold on
stairs(Xm(:,1),'b--');
title('Comparação das Estimações de X')
legend({'Kalman','Odometria'});
ylabel('X(m)'); xlabel('t');

figure
stairs(Yk(:,1),'r--');
hold on
stairs(Ym(:,1),'b--');
title('Comparação das Estimações de Y')
legend({'Kalman','Odometria'});
ylabel('Y(m)'); xlabel('t');