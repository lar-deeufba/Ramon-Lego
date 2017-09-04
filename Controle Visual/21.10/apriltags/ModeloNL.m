function [P1] = ModeloNL(P0, Wd, We, Ts, l ,r)
% [P1] = ModeloNL(P0, v, w, ts, l ,r)
% funcao que contem o modelo linearizado do robo diferencial
%   P0 = [x; y; theta] posicao atual do robo
%   P1 = [x1; y1; theta1] posicao apos aplicar os sinais de controle
%   v = velocidade linear [m/s]
%   w = velocidade angular [rad/s]
%   wd = velocidade angular da roda direita
%   we = velocidade angular da roda esquerda
%   ts = periodo de amostragem [s]
%   l = [m] espaco entre a roda e o centro do robo
%   r = [m] raio das rodas

    % Define as predicoes anteriores
    x_esta = P0(1,1);
    y_esta = P0(2,1);
    theta_esta = P0(3,1);
    % Inclui a amostragem
    Wd = Wd*Ts; 
    We = We*Ts; 
    % Atualiza o modelo
    L = (Wd*r + We*r)/2;                        % Variavel auxiliar
    Omega = (Wd*r - We*r)/(2*l);                % Variavel auxiliar
    % Testa se Omega � zero
    if Omega~=0
        x_est = x_esta + (L/Omega)*(sin(theta_esta + Omega) - ...
                                                          sin(theta_esta));
        y_est = y_esta - (L/Omega)*(cos(theta_esta + Omega) - ...
                                                          cos(theta_esta));
        theta_est = theta_esta + Omega;
    else
        x_est = x_esta + L*cos(theta_esta);
        y_est = y_esta + L*sin(theta_esta);
        theta_est = theta_esta;
    end
    % Calcula a predicao do estado
    P1 = [x_est; y_est; theta_est]; 
end

