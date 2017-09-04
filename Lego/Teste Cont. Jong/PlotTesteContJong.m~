clear all; close all; clc
%% Leituras
% Lê os arquivos
load('Dados.txt');
% Cria vetor de tempo
[t nn] = size(Dados);
t = [1:t];
% Posições Estimadas
Xest = Dados(:,1);
Yest = Dados(:,2);
Theta = Dados(:,9);
% Vetor com as velocidades desejadas
Ref_d = Dados(:,3);
Ref_e = Dados(:,4);
% Velocidades reais
W = Dados(:,5:6);
% Sinais de controle d
U = Dados(:,7:8);
% Tamanho do vetor
[m, n] = size(W);
qt = m;

%% Gráficos
% Gera o gráfico de posição
% Gera graficos da posiçao 
h1 = plot(Xest(1,1),Yest(1,1),'gs','MarkerSize',6);
hold on
plot(Xest(1:qt-1,1),Yest(1:qt-1,1),'ks','MarkerSize',4);
h2 = plot(Xest(qt,1),Yest(qt,1),'ks','MarkerSize',6);
h3 = plot(Xest(qt,1),Yest(qt,1),'ms','MarkerSize',6);
%axis([-3 3 0 2.5]);
grid;
title('Posicao Estimada (m)')
xlabel('X'); ylabel('Y');
legend([h1, h3, h2],{'Posicao Inicial','Posicao Final','Progressão'});
figure
% Gráfico do ângulo
stairs(Theta(:,1)*180./pi,'k--');
title('Ângulo estimado (graus)')
ylabel('\Theta'); xlabel('t');
figure

Ref_d = (Ref_d./127)*17.3398;
Ref_e = (Ref_e./127)*17.3398;
% % Gera gráficos da roda Direita
% plot(t,W(:,1),'ro',t,Ref_d,'bo');
% hold on;
% stairs(t,W(:,1),'r');
% stairs(t,Ref_d,'b');
% title('Velocidade da Roda Direita');
% xlabel('amostra'); ylabel('rad/s');
% legend('Velocidade Medida','Referencia');
% % figure
% % % Gráficos dos sinais de controle aplicados
% % plot(t,U(:,1),'bo');
% % stairs(t,U(:,1),'b');
% % title('Sinal de Controle da Roda Direita');
% 
% % Gera gráficos da roda Esquerda
% figure
% plot(t,W(:,2),'ro',t,Ref_e,'bo');
% hold on;
% stairs(t,W(:,2),'r');
% stairs(t,Ref_e,'b');
% title('Velocidade da Roda Esquerda');
% xlabel('amostra'); ylabel('rad/s');
% legend('Velocidade Medida','Referencia');
% figure
% % % Gráficos dos sinais de controle aplicados
% % plot(t,U(:,2),'bo');
% % stairs(t,U(:,2),'b');
% % title('Sinal de Controle da Roda Esquerda')
% % figure
% 
% Plots para comparação
plot(t,Ref_d,'bo'); hold on;
plot(t,Ref_e,'ro'); 
stairs(t,Ref_d,'b'); 
stairs(t,Ref_e,'r'); 
title('Velocidades Desejadas');
xlabel('amostra'); ylabel('rad/s');
legend('Roda Direita','Roda Esquerda');
 
figure
plot(t,W(:,1),'bo'); hold on;
plot(t,W(:,2),'ro'); 
stairs(t,W(:,1),'b'); 
stairs(t,W(:,2),'r'); 
title('Velocidades Medidas');
xlabel('amostra'); ylabel('rad/s');
legend('Roda Direita','Roda Esquerda');

figure
plot(t,U(:,1),'bo'); hold on;
plot(t,U(:,2),'ro'); 
plot(t,Dados(:,10),'go'); 
stairs(t,U(:,1),'b'); 
stairs(t,U(:,2),'r');
stairs(t,Dados(:,10),'go');
title('Erros');
xlabel('amostra');
legend('E_X','E_y','E_theta');
 
 
 
