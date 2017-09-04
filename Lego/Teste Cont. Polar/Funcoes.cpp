/*
 * @file Funcoes.cpp
 * @brief A ser usada com o apriltags_demo.cpp.
 * @author: Ramon O. Fernandes
 * @versão 1.0
 *
 * Contém as funções que serão usadas durante a execução de apriltags_demo.cpp
 */

#include <cstdio>
#include <iostream> 
#include <fstream> 
#include <cstdlib> 
#include <iostream>
#include <stdlib.h>
#include <cstring>
#include <list>
#include <vector>
#include <cmath>
// Biblioteca de tempo do linux
#include <sys/time.h>
// Biblioteca para obter imagens da câmera e mostrá-las na tela
#include "opencv2/opencv.hpp"
// Funcionalidades do OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui_c.h>

using namespace cv;
using namespace std;
/*------------------------------------Constantes-----------------------------------*/
const double pi = 3.14159265358979323846;
const double r = 0.028;	 					  // Raio da roda (m)
const double rd = 0.028; 					  // Raio da roda (m)
const double re = 0.028;					  // Raio da roda (m)
const double l = 0.12;      	     // Distância entre o centro do eixo e a roda (m)
const double Wmax = 17.3398;			    	 // Velocidade Máxima (rad/s)
const double Ts = 0.6;					 // Periodo de amostragem (s)
// Ganhos do controlador polar
const double Kalfa = 0.15;
const double Kbeta = 0.005;
const double Kr = 0.08; 

/*  V2Winv  calcula a matriz de transformação de velocidade global para velocida-
    de das rodas para o Lego EV3.Função criada para evitar a repetição do cálculo 
    de uma matriz constante.                                                       */ 
Mat V2Winv(){
    double Rld = rd/(2*l);
    double Rle = re/(2*l);
    Mat A = (Mat_<double>(2,2) << rd/2, re/2, Rld, -Rle);
    A = A.inv();

    if(A.empty()){
        cout << "Erro na definição da Matriz A\n " ;
        exit(2);        
    }
    else
        return A;
}

/*-------------------------------------Funções-------------------------------------*/
/*---------------------------------------de----------------------------------------*/
/*------------------------------------Uso Geral------------------------------------*/

/*  Obtem o tempo do sistema                                                       */
double tic(){
    struct timeval t;
    gettimeofday(&t, NULL);
    return ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
}

/*  Normaliza o ângulo para o internalo [-pi,pi]                                   */
inline double standardRad(double t){
    if (t >= 0.) 
        t = fmod(t+pi, 2*pi) - pi;
    else 
        t = fmod(t-pi, -2*pi) + pi;
    return t;
}

/*  Converte as matrizes de rotação para os ângulos de Euler                       */
void wRo_to_euler(const Mat& wRo, double& yaw, double& pitch, double& roll){
    double ang = atan2(wRo.at<double>(1,0), wRo.at<double>(0,0));
    yaw = standardRad(ang);
    double c = cos(yaw);
    double s = sin(yaw);
    pitch = standardRad(atan2(-wRo.at<double>(2,0), wRo.at<double>(0,0)*c + wRo.at<double>(1,0)*s));
    roll  = standardRad(atan2(wRo.at<double>(0,2)*s - wRo.at<double>(1,2)*c, -wRo.at<double>(0,1)*s + wRo.at<double>(1,1)*c));
}

/* Função que salva os dados em um arquivo txt                                     */
void Historico(double arg1, double arg2, double arg3, double arg4, double arg5, 
	       double arg6, double arg7, double arg8, double arg9, double arg10,
	                                                        const char* name){ 
  FILE *arquivo;
  arquivo = fopen(name,"a+");  // abre o arquivo para escrita 
  fprintf(arquivo, "%f  %f %f %f %f %f %f %f %f %f \n", arg1, arg2, arg3, arg4, arg5,
                                                       arg6, arg7, arg8, arg9, arg10);
  fclose(arquivo);  // fecha o arquivo para escrita
} 

/* Função que limpa os dados salvos em um arquivo txt                              */
void LimpaHistorico(const char* name){ 
    FILE *arquivo;
    arquivo = fopen(name,"w");  // abre o arquivo para escrita 
} 

/*-------------------------------------Funções-------------------------------------*/
/*---------------------------------------de----------------------------------------*/
/*------------------------------------Controle-------------------------------------*/

/* [Wd,We] = Vdiff(Vx,Wz) tem como objetivo calcular as velocidades ângular    das
    rodas direita e esquerda para a configuração diferencial, com base nos parâme-
    tros: velocidade na direção x, Vx; velocidade ângular, Wz. Wd e We são   dadas 
    em valores de 7 bits proporcionais a Wmax                                      */
Mat Vdiff(const double& Vx, const double& Wz){
    Mat A = V2Winv(); 
    // Cálculo da velocidade (Modelo Robô Diferencial)
    double Vr = abs(Vx);                                         // Velocidade global
    // Cria matrizes para o cálculo
    Mat V = (Mat_<double>(2,1) << Vx, Wz);
    // Cálculo
    Mat W  = A*V;
    double Wd = W.at<double>(0,0);
    double We = W.at<double>(1,0);
    // Transforma as velocidades em 7bits e limitam o valor 
    Wd = (Wd/Wmax)*127;       
    if (abs(Wd)>=127)        
        Wd = (Wd/abs(Wd))*127;
    We = (We/Wmax)*127;       
    if (abs(We)>=127)        
        We = (We/abs(We))*127;
    // Recompõe a velocidade angular
    W.at<double>(0,0) = Wd;
    W.at<double>(1,0) = We;
    if(W.empty()){
        cout << "Erro em Vdiff " ;
        exit(3);        
    }
    else{
        return W;
    }
}

/* Direcao define o sentido da velocidade do controlador 
    direcao = Direcao(Vx,Vy,Wz,R,L) tem como objetivo calcular o sentido do movi-
    mento dado pelo controlador nos parâmetros: variação na direção x, Vx; varia-
    ção na direção y, Vy;  ângulo do robô, theta.                                 */
double Direcao(const double& deltax, const double& deltay, const double& theta){
    double direcao;
    double alfa = standardRad(atan2(-deltay, -deltax)) - theta;
    if ((abs(alfa)< pi/2))
        direcao = 1;
    else
        direcao = -1;
    return direcao;
}

/* Vdiff_p calcula os estados para um robô diferêncial segundo a forma de cadeia
    [r,alfa,beta,d] = Vdiff_p(Vx,Vy,Wz,R,L) tem como objetivo calcular os estados
    nos parâmetros: variação na direção x, Vx; variação na direção y, Vy;  ângulo 
    do robô, theta. Os estados são definidos: distância à posição desejada, r; o-
    rientação inicial, alfa; orientação desejada, beta;e direção do movimento.     */
Mat Vdiff_p(const double& deltax, const double& deltay, const double& theta){
    // Cálculo da velocidade (Modelo Robô Diferencial)
    double direcao, beta;
    double rr = sqrt(deltax*deltax + deltay*deltay);                      
    double alfa = standardRad(atan2(-deltay, -deltax)) - theta;
    if (abs(alfa) < pi/2){
        direcao = 1;
	beta = -theta - alfa; 
    }else{
	direcao = -1;
	alfa = standardRad(atan2(deltay, deltax)) - theta;
        beta = -theta - alfa;
    }    
    Mat K = (Mat_<double>(4,1) << alfa, beta, rr, direcao);
    if (K.at<double>(2,0)<0 || (direcao*direcao!=1)){
        cout << "Erro na função Vdiff_p\n " ;
        exit(5);        
    }
    else{
        return K;
    }
}

/* Controle( deltax, deltay, theta) é o valor das velocidades de controle Wd,  e        
    We, baseada nas funções anteriormente definidas: Vdiff e Vdiff_p               */
Mat Controle(const double& deltax, const double& deltay, const double& theta, 
	                         const double& theta_d, const double& direcao){
    // Escreve as equações na forma "polar"
    const Mat Pp = Vdiff_p( deltax, deltay, theta);
    // Calcula os sinais de controle baseado na forma "polar"
    double UWp = (Kalfa*Pp.at<double>(0,0) + (-Kbeta)*(Pp.at<double>(1,0) + theta_d));
    double UVp = direcao*Kr*Pp.at<double>(2,0);
    // Transforma para W do robô
    Mat Ww = Vdiff(UVp, UWp);
    return Ww;
} 

/* Kalman( x, y, theta, Wd, We, mi) implementa um filtro Kalman estendido baseado          
    no modelo de um robô diferencial tendo como entradas as imformaçãoes de odome-
    tria. No filtro é feita a fusão entre os dados do controle visual e odometria  */
Mat Kalman(const double& X, const double& Y, const double& theta, 
	   const double& Wd, const double& We, const Mat& mi_a, const Mat& Sigma_a){
    //Definição de variáveis
    double x_est, y_est, theta_est;
    double g13, g23, k1, k2, k3, v11, v12, v21, v22, v31, v32; 
    // Covariância do Sensor Visual
    double err_x = 0.05;                                                       // (m)
    double err_y = 0.05;                                                       // (m)
    double err_theta = 0.18;                                         // 10.3132 graus
    // Matriz de covariancia
    Mat Qk = (Mat_<double>(3,3) << err_x*err_x,    0,             0,
                                        0   , err_y*err_y,        0,
                                        0   ,      0   , err_theta*err_theta);
    // Estados iniciais
    double x_esta = mi_a.at<double>(0,0);
    double y_esta = mi_a.at<double>(1,0);
    double theta_esta = mi_a.at<double>(2,0);
    // Deslocamento angular baseado na velocidade
    double Wd_est = Wd*Ts; 
    double We_est = We*Ts; 
    // Atualiza o modelo
    double L = (Wd_est*r + We_est*r)/2;                        	 // Variável auxiliar
    double Omega = (Wd_est*r - We_est*r)/(2*l);                  // Variável auxiliar
    // Testa se Omega é zero
    if(Omega!=0){
	x_est = x_esta - (L/Omega)*(sin(theta_esta + Omega) - sin(theta_esta));
        y_est = y_esta + (L/Omega)*(cos(theta_esta + Omega) - cos(theta_esta));
        theta_est = theta_esta + Omega;
    }else{
        x_est = x_esta - L*cos(theta_esta);
        y_est = y_esta - L*sin(theta_esta);
        theta_est = theta_esta;
    }
    // Calcula a predição da média dos estados
    Mat mi_barra = (Mat_<double>(3,1) <<   x_est,
		                           y_est,
		                         theta_est); 
    // Considera singularidades no cálculo das matrizes de Predição
    if(Omega!=0){
        g13 = -(L/Omega)*(cos(theta_esta + Omega) - cos(theta_esta));
        g23 = -(L/Omega)*(sin(theta_esta + Omega) - sin(theta_esta));
    
        k1 = (r*(Wd_est + We_est))/((2*l)*(Wd_est - We_est));
        k2 = theta_esta + r*((Wd_est - We_est)/(2*l));
        k3 = ((2*l)*We_est)/(2*(r*(Wd_est - We_est)/(2*l))*(r*(Wd_est - We_est)/(2*l)));
        v11 = -k1*cos(k2) + k3*(sin(k2) - sin(theta_esta));
        v12 = k1*cos(k2) - k3*(sin(k2) - sin(theta_esta));
        v21 = -k1*sin(k2) + k3*(-cos(k2) + cos(theta_esta));
        v22 = k1*sin(k2) - k3*(-cos(k2) + cos(theta_esta));
        v31 = r/(2*l);
        v32 = -r/(2*l);
    }else{
        g13 = (L)*sin(theta_esta);
        g23 = -(L)*cos(theta_esta);
    
        v11 = -r*cos(theta_esta)/2;
        v12 = -r*cos(theta_esta)/2;
        v21 = -r*sin(theta_esta)/2;
        v22 = -r*sin(theta_esta)/2;
        v31 = 0;
        v32 = 0;
    }
    // Calcula as matrizes de Predição
    Mat Gk = (Mat_<double>(3,3) <<  1, 0, g13,
                                    0, 1, g23,
                                    0, 0,  1  );
    Mat Vk = (Mat_<double>(3,2) << v11,  v12,
                                   v21,  v22,
                                   v31,  v32);
    // Covariância do encoder, considerado proporcional à leitura
    double alfa1 = 0.01;
    double alfa2 = 0.01;
    Mat Mk = (Mat_<double>(2,2) << (alfa1*(Wd_est))*(alfa1*(Wd_est)),             1,
                                                   1,          (alfa2*abs(We_est))*(alfa2*(We_est)));
    // Predição da variância
    Mat Sigma_barra;
    Sigma_barra = (Gk*Sigma_a*Gk.t()) + (Vk*Mk*Vk.t());

    // Sensor visual modelado como contendo um erro gaussiano e varia da mesma
    // forma que o robô em relação à variação dos estados
//    Mat Hk = Gk;
    Mat Hk = (Mat_<double>(3,3) <<  1, 0, 0,
                                    0, 1, 0,
                                    0, 0, 1  );
    // Modelo do sensor visual
    Mat  err = (Mat_<double>(3,1) <<   err_x,           // Matriz com os erros
		                       err_y,
		                     err_theta); 
    Mat h;
    h = Hk*mi_barra + err;
    // Ganho de Kalman
    Mat Aux;
    Aux = (Hk*Sigma_barra*Hk.t() + Qk);
    Mat K;
    K = Sigma_barra*Hk.t()*Aux.inv();
    // Atualizações
    Mat I = (Mat_<double>(3,3) <<  1, 0, 0,
                                   0, 1, 0,
                                   0, 0, 1 );
    Mat Sigma = (I - K*Hk)*Sigma_barra;

    Mat  Z = (Mat_<double>(3,1) <<   X,
		                     Y,
		                   theta); 
    Mat mi = mi_barra + K*(Z - h);
    // Cria a matriz de saídas
    Mat Res(3, 4, CV_64F);
    Res.at<double>(0,0) = mi.at<double>(0,0);
    Res.at<double>(1,0) = mi.at<double>(1,0);
    Res.at<double>(2,0) = mi.at<double>(2,0);
    Res.at<double>(0,1) = Sigma.at<double>(0,0);
    Res.at<double>(1,1) = Sigma.at<double>(1,0);
    Res.at<double>(2,1) = Sigma.at<double>(2,0);
    Res.at<double>(0,2) = Sigma.at<double>(0,1);
    Res.at<double>(1,2) = Sigma.at<double>(1,1);
    Res.at<double>(2,2) = Sigma.at<double>(2,1);
    Res.at<double>(0,3) = Sigma.at<double>(0,2);
    Res.at<double>(1,3) = Sigma.at<double>(1,2);
    Res.at<double>(2,3) = Sigma.at<double>(2,2);
  
    return Res;   
} 

/* Odometria( theta, Wd, We, mi) implementa um filtro Kalman estendido baseado          
    no modelo de um robô diferencial tendo como entradas as imformaçãoes de odome-
    tria                                                                           */
Mat Odometria( const Mat& mi_a, const double& Wd, const double& We){
    //Definição de variáveis
    double x_est, y_est, theta_est;
    // Estados iniciais
    double x_esta = mi_a.at<double>(0,0);
    double y_esta = mi_a.at<double>(1,0);
    double theta_esta = mi_a.at<double>(2,0);
    // Deslocamento angular baseado na velocidade
    double Wd_est = Wd*Ts; 
    double We_est = We*Ts; 
    // Atualiza o modelo
    double L = (Wd_est*rd + We_est*re)/2;                      	 // Variável auxiliar
    double Omega = (Wd_est*rd - We_est*re)/(2*l);                // Variável auxiliar
    // Testa se Omega é zero
    if(Omega!=0){
	x_est = x_esta + (L/Omega)*(sin(theta_esta + Omega) - sin(theta_esta));
        y_est = y_esta - (L/Omega)*(cos(theta_esta + Omega) - cos(theta_esta));
        theta_est = theta_esta + Omega;
    }else{
        x_est = x_esta + L*cos(theta_esta);
        y_est = y_esta + L*sin(theta_esta);
        theta_est = theta_esta;
    }
    if(abs(theta_est)>=2*pi)
	theta_est = (theta_est - floor(abs(theta_est)/(2*pi))*2*pi*(abs(theta_est)/theta_est));
    // Calcula a predição da média dos estados
    Mat mi_barra = (Mat_<double>(3,1) <<   x_est,
		                           y_est,
		                         theta_est); 
    return mi_barra;   
} 
