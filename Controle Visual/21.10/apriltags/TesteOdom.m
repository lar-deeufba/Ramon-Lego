clear all; close all; clc

% Le os arquivos
load('Dados.txt');
load('Dadosk.txt');
load('DadosZ.txt');

% Dados sensor
rZ = DadosZ(:,1);
% Artifício que confere se o problema são os sinais gigantes obtidos
for i=1:size(rZ,1)
    if rZ(i)>=4
        rZ(i)=4;
    end
end       
fiZ = DadosZ(:,2);
iZ = DadosZ(:,3);
mZ = DadosZ(:,4);
% Posicoes estimadas pela odometria embarcada
Xest = Dados(:,1);
Yest = Dados(:,2);
Theta = Dados(:,3);
% Velocidades reais
W = Dadosk(:,5:6);
% Sinais de controle d
U = Dadosk(:,7:8);
% Tamanho dos vetores
qt = length(Xest);
q = length(rZ);
P1 = [Xest(1); Yest(1); Theta(1)];
% Estimacao odometrica
for i=1:qt
    Xm(i,1) = P1(1);
    Ym(i,1) = P1(2);
    Theta_m(i,1) = P1(3);
    P1 = ModeloNL(P1, W(i,1), W(i,2), 0.6, 0.066, 0.0278);
end
% Simula o filtro kalman
mi = [2,0,pi];
Sigma = zeros(3,3);
r = 0.0278;
l = 0.066;
Ts = 0.6;
ii = 1;
for i = 1:qt
    % Ajusta os dados dependendo da quantidade de alvos encontrada
    Z = zeros(3,1);     Loc = zeros(3,1);
    a = ii;
    while ii<a+mZ(a)
        Zi = [ rZ(ii)
              fiZ(ii)
               iZ(ii) ];
        Z = [Z Zi];
        switch iZ(ii)
            case 1
                Loci = [0;0;0];
            case 2
                Loci = [-0.5;0;0];
            case 3
                Loci = [0;0.5;0];
            case 4
                Loci = [0.75;2;0];
        end
        Loc = [Loc Loci];
        ii = ii + 1;
    end
    Z(:,1) = [];
    Loc(:,1) = [];
    Kal = VKalman(mi, Sigma, W(i,1), W(i,2), Z, Loc, r, l, Ts);
    mi = Kal(:,1);
    Sigma = Kal(:,2:4);
    Xk(i) = mi(1);
    Yk(i) = mi(2);
    Theta_k(i) = mi(3);
end
qk = length(Xk);
%% Gera graficos
% Gera graficos da posicao 
h1 = plot(Yest(1,1),Xest(1,1),'gs','MarkerSize',6);
hold on
plot(Yest(1:qt-1,1),Xest(1:qt-1,1),'ks','MarkerSize',4);
h2 = plot(Yest(qt,1),Xest(qt,1),'ks','MarkerSize',6);
h3 = plot(Yest(qt,1),Xest(qt,1),'ms','MarkerSize',6);
plot(Yest,Xest,'k','LineWidth', 0.1);
plot(Yk(1,1),Xk(1,1),'go','MarkerSize',6);
plot(Yk(1:qk-1),Xk(1:qk-1),'ro','MarkerSize',4);
h4 = plot(Yk(qk),Xk(qk),'ro','MarkerSize',6);
plot(Yk(qk),Xk(qk),'mo','MarkerSize',6);
plot(Yk,Xk,'r','LineWidth', 0.1);
plot(Ym(1,1),Xm(1,1),'g*','MarkerSize',6);
plot(Ym(1:qk,1),Xm(1:qk,1),'b*','MarkerSize',4);
h6 = plot(Ym(qk,1),Xm(qk,1),'b*','MarkerSize',6);
plot(Ym(qk,1),Xm(qk,1),'m*','MarkerSize',6);
plot(Ym,Xm,'b','LineWidth', 0.1);
%axis([-1 1 0 2.5]);
grid;
title('Posicao Estimada (m)')
xlabel('Y'); ylabel('X');
legend([h1, h3, h2, h4, h6],{'Posicao Inicial','Posicao Final',...
                            'Visao','Kalman','Odometria'});
figure

% Grafico do Angulo
stairs(Theta(:,1)*180./pi,'k--');
hold on
stairs(Theta_k(:)*180./pi,'r--');
stairs(Theta_m(:,1)*180./pi),'b--';
title('Angulo estimado (graus)')
legend({'Visao','Kalman','Odometria'});
ylabel('\Theta'); xlabel('t');
