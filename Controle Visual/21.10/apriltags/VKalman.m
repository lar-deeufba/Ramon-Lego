function [out] = VKalman(mi, Sigma_a, Wd, We, Z, Loc, r, l, Ts)
%VKalman e a implementacao de um filtro de kalman para um controle visual
% PVBS com odometria segundo o modelo de Thrun.
%   mi => Predicao da media dos estados anterior
%   Sigma_a => Predicao da matriz de covariancia anterior
%   Wd => Velocidade da roda direita (rad/s)
%   We => Velocidade da roda esquerda (rad/s)
%   Z => Dados da visao
%   Loc => posicao conhecida dos marcos
%   r => raio da roda do robo (m)
%   l => metade da distancia entre as rodas do robo (m)
%   Ts => Periodo de amostragem (s)

    % Covariancia do Sensor Visual
    err_x = 0.2;          
    err_y = 0.2;         
    err_Tagi = 1;                                            
    % Covariancia do Modelo Odometrico:
    epslon_d = 0.3;
    epslon_e = 0.3; 
    % Matrizes de covariancia
    Qk = [ err_x^2,    0,         0
              0   , err_y^2,      0
              0   ,    0   , err_Tagi^2];
    R = [ epslon_d^2,       0  
               0    ,  epslon_e^2];
    % Define as predicoes anteriores
    x_esta = mi(1);
    y_esta = mi(2);
    theta_esta = mi(3);
    % Inclui a amostragem
    Wd_est = Wd*Ts; 
    We_est = We*Ts; 
    % Atualiza o modelo
    L = (Wd_est*r + We_est*r)/2;                        % Variavel auxiliar
    Omega = (Wd_est*r - We_est*r)/(2*l);                % Variavel auxiliar
    % Testa se Omega é zero
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
    % Calcula a predicao da media do estado
    mi_barra = [x_est; y_est; theta_est]; 
    % Considera singularidades no calculo das matrizes de Predicao
    if  Omega~=0
        g13 = (L/Omega)*(cos(theta_esta + Omega) - cos(theta_esta));
        g23 = -(L/Omega)*(sin(theta_esta + Omega) - sin(theta_esta));
    
        k1 = (r*(Wd_est + We_est))/((2*l)*(Wd_est - We_est));
        k2 = theta_esta + r*((Wd_est - We_est)/(2*l));
        k3 = ((2*l)*We_est)/(2*(r*(Wd_est - We_est)/(2*l))^2);
        k4 = ((2*l)*Wd_est)/(2*(r*(Wd_est - We_est)/(2*l))^2);
        v11 = k1*cos(k2) - k3*(sin(k2) - sin(theta_esta));
        v12 = -k1*cos(k2) + k4*(sin(k2) - sin(theta_esta));
        v21 = k1*sin(k2) - k3*(-cos(k2) + cos(theta_esta));
        v22 = -k1*sin(k2) + k4*(-cos(k2) + cos(theta_esta));
        v31 = r/(2*l);
        v32 = -r/(2*l);
    else
        g13 = -(L)*sin(theta_esta);
        g23 = (L)*cos(theta_esta);
    
        v11 = r*cos(theta_esta)/2;
        v12 = r*cos(theta_esta)/2;
        v21 = r*sin(theta_esta)/2;
        v22 = r*sin(theta_esta)/2;
        v31 = 0;
        v32 = 0;
    end
    % Calcula as matrizes de Predicao
    Gk = [ 1, 0, g13
           0, 1, g23
           0, 0,  1 ];
    Vk = [ v11  v12
           v21  v22
           v31  v32 ];
    % Relacionado ao erro do encoder, considerado proporcional a leitura
    alfa1 = 0.002;
    alfa2 = 0.002;
    Mk = [ (alfa1*(Wd_est))^2,           0
                 0,           (alfa2*abs(We_est))^2 ];
    % Predicao da variancia
    Sigma_barra = Gk*Sigma_a*Gk' + Vk*Mk*Vk';
    % Sensor visual modelado como em Thrun
    m = size(Z,2);
    ii = 1;
    Aux_mi = [0; 0; 0];
    Aux_Sigma = eye(3,3);
    while(ii<m)
        delta_x = Loc(1,ii) - x_est;
        delta_y = Loc(2,ii) - y_est;
        q = delta_x^2 + delta_y^2;
        zi = [           (q)^(1/2)
               atan2(delta_y,delta_x)-theta_est 
                            1                   ];        
        Hi = [ -delta_x/((q)^(1/2)) -delta_y/((q)^(1/2)) 0
                     delta_y/(q)         -delta_x/(q)   -1
                          0                    0         0 ];
        % Ganho de Kalman
        Ki = Sigma_barra*Hi'*inv(Hi*Sigma_barra*Hi' + Qk);
        % Atualizacoes
        Aux_Sigma = (eye(3) - Ki*Hi)*Aux_Sigma;
        Aux_mi = Ki*(Z(:,1) - zi) + Aux_mi;
        ii = ii + 1;
    end
    mi = mi_barra + Aux_mi;  
    Sigma = Aux_Sigma*Sigma_barra;   
    out = [mi Sigma];
end

