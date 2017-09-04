clear all; close all; clc

% Lê os arquivos
load('Dados.txt');
load('Dadosk.txt');
load('DadosSemI.txt');

% Posicoes estimadas pela visão
%Xest = Dados(:,7);
%Yest = Dados(:,8);
%Theta = Dados(:,9);
% Posicoes filtro kalman
Xk = Dadosk(:,1);
Yk = Dadosk(:,2);
Theta_k = Dadosk(:,9);
% Progressão do erro
Ex = Dados(:,4);
Ey = Dados(:,5);
Etheta = Dados(:,6);
% Posicoes estimadas pela visão
if length(DadosSemI)>0
% Cria vetor de tempo
    t1 = Dadosk(:,10);
    t2 = DadosSemI(:,10);
    t = [t1;t2];
    Xsi = DadosSemI(:,1);
    Ysi = DadosSemI(:,2);
    % Vetor com as velocidades desejadas
    Ref_d1 = Dadosk(:,3);
    Ref_d2 = DadosSemI(:,3);
    Ref_d = [Ref_d1;Ref_d2];
    Ref_e1 = Dadosk(:,4);
    Ref_e2 = DadosSemI(:,4);
    Ref_e = [Ref_e1;Ref_e2];
    % Velocidades reais
    W1 = Dadosk(:,5:6);
    W2 = DadosSemI(:,5:6);
    W = [W1;W2];
    % Sinais de controle d
    U1 = Dadosk(:,7:8);
    U2 = DadosSemI(:,5:6);
    U = [U1;U2];
    % Tamanho do vetor
    qt = length(Xest);
    qk = length(Xk);
    qsi =length(Xsi);
    q = length(W);
    P1 = [Xest(1); Yest(1); Theta(1)];
else
    % Cria vetor de tempo
    t1 = Dadosk(:,10);
    t2 = zeros(length(Xk),1);
    t = [t1;t2];
    Xsi = zeros(length(Xk),1);
    Ysi = zeros(length(Xk),1);
    % Vetor com as velocidades desejadas
    Ref_d1 = Dadosk(:,3);
    Ref_d2 = zeros(length(Xk),1);
    Ref_d = [Ref_d1;Ref_d2];
    Ref_e1 = Dadosk(:,4);
    Ref_e2 = zeros(length(Xk),1);
    Ref_e = [Ref_e1;Ref_e2];
    % Velocidades reais
    W1 = Dadosk(:,5:6);
    W2 = zeros(length(Xk),2);
    W = [W1;W2];
    % Sinais de controle d
    U1 = Dadosk(:,7:8);
    U2 = zeros(length(Xk),2);
    U = [U1;U2];
    % Tamanho do vetor
    qt = length(Xest);
    qk = length(Xk);
    qsi =length(Xsi);
    q = length(W);
    P1 = [Xest(1); Yest(1); Theta(1)];
end
% Estimação odométrica
for i=1:q
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
h1 = plot(Yest(1,1),Xest(1,1),'gs','MarkerSize',6);
hold on
plot(Yest(1:qt-1,1),Xest(1:qt-1,1),'ks','MarkerSize',4);
h2 = plot(Yest(qt,1),Xest(qt,1),'ks','MarkerSize',6);
h3 = plot(Yest(qt,1),Xest(qt,1),'ms','MarkerSize',6);
plot(Yest,Xest,'k','LineWidth', 0.1);
plot(Yk(1,1),Xk(1,1),'go','MarkerSize',6);
plot(Yk(1:qk-2,1),Xk(1:qk-2,1),'ro','MarkerSize',4);
h4 = plot(Yk(qk-1,1),Xk(qk-1,1),'ro','MarkerSize',6);
plot(Yk(qk-1,1),Xk(qk-1,1),'mo','MarkerSize',6);
plot(Yk,Xk,'r','LineWidth', 0.1);
plot(Ym(1,1),Xm(1,1),'g*','MarkerSize',6);
plot(Ym(1:q,1),Xm(1:q,1),'b*','MarkerSize',4);
h6 = plot(Ym(q,1),Xm(q,1),'b*','MarkerSize',6);
plot(Ym(q,1),Xm(q,1),'m*','MarkerSize',6);
plot(Ym,Xm,'b','LineWidth', 0.1);
plot(Ysi(1,1),Xsi(1,1),'g^','MarkerSize',6);
plot(Ysi(1:qsi,1),Xsi(1:qsi,1),'c^','MarkerSize',4);
h7 = plot(Ysi(qsi,1),Xsi(qsi,1),'c^','MarkerSize',6);
plot(Ysi(qsi,1),Xsi(qsi,1),'m^','MarkerSize',6);
plot(Ysi,Xsi,'c','LineWidth', 0.1);
%axis([-1 1 0 2.5]);
grid;
title('Posicao Estimada (m)')
xlabel('Y'); ylabel('X');
legend([h1, h3, h2, h4, h6, h7],{'Posicao Inicial','Posicao Final',...
                            'Visao','Kalman','Odometria','Sem Imagem'});
figure

% Gráfico do ângulo
stairs(Theta(:,1)*180./pi,'k--');
hold on
stairs(Theta_k(:,1)*180./pi,'r--');
stairs(Theta_m(:,1)*180./pi),'b--';
title('Ângulo estimado (graus)')
legend({'Visao','Kalman','Odometria'});
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
stairs(Xest(:,1),'k--');
hold on
stairs(Xk(:,1),'r--');
stairs(Xm(:,1),'b--');
title('Comparação das Estimações de X')
legend({'Visao','Kalman','Odometria'});
ylabel('X(m)'); xlabel('t');

figure
stairs(Yest(:,1),'k--');
hold on
stairs(Yk(:,1),'r--');
stairs(Ym(:,1),'b--');
title('Comparação das Estimações de Y')
legend({'Visao','Kalman','Odometria'});
ylabel('Y(m)'); xlabel('t');