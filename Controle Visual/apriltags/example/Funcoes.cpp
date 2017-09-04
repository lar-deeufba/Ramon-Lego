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
/*  Para evitar repetições, duplicidades e uma boa inicialização, são definida as  
    constantes globais.                                                            */
const double pi = 3.14159265358979323846;
const double r = 0.0278; 					  // Raio da roda (m)
const double rd = 0.0278; 					  // Raio da roda (m)
const double re = 0.0278;					  // Raio da roda (m)
const double l = 0.066;      	     // Distância entre o centro do eixo e a roda (m)
const double Wmax = 17.3398;			    	 // Velocidade Máxima (rad/s)
const double Ts = 0.6;					 // Periodo de amostragem (s)
// Matriz de transformação de velocidade global para velocidade das rodas do Lego EV3
Mat Aux = (Mat_<double>(2,2) << rd/2, re/2, rd/(2*l), -re/(2*l));
const Mat A = Aux.inv();
// Parâmetros K_alfa, K_beta, e K_r 
const double Kalfa = 0.15;
const double Kbeta = 0.002;
const double Kr = 0.1;

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
double standardRad(double t){
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

/* Função que calcula o produto de kronecker                                       */
Mat Kron(const Mat& A, const Mat& B){
    Mat temp, res;
    for (int i=0; i<(B.rows); i++)
    {
	for (int j=0; j<(A.rows); j++)
	{
	    for(int k = 0; k<(B.cols); k++){
		Mat a = A.row(j) * B.at<double>(i, k);
		if(k>0)
		  hconcat(temp, a, temp);
		else
		  temp = a;		  
	    }
	    res.push_back(temp);
	}	
    }  
    return res;
}
/*-------------------------------------Funções-------------------------------------*/
/*---------------------------------------de----------------------------------------*/
/*------------------------------------Controle-------------------------------------*/

/* [Wd,We] = Vdiff(Vx,Vy,Wz) tem como objetivo calcular as velocidades ângular
    das rodas direita e esquerda para a configuração diferencial,    com base nos      
    parâmetros: velocidade na direção x, Vx; velocidade na direção y, Vy; veloci-
    dade ângular, Wz; Matriz de transformação, A;   e velocidade máxima das rodas   
    Wmax. Wd e We são dadas em valores de 7 bits proporcionais a Wmax              */
Mat Vdiff(const double& Vx, const double& Vy, const double& Wz){
    // Cálculo da velocidade (Modelo Robô Diferencial)
    double Vr = sqrt(Vx*Vx);                                     // Velocidade global
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
    double alfa = -standardRad(standardRad(atan2(-deltay, -deltax)) - standardRad(theta));
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
    double alfa = Pp.at<double>(0,0);
    double beta = Pp.at<double>(1,0);
    double rr =  Pp.at<double>(2,0);
    // Calcula os sinais de controle baseado na forma "polar"
    double UWp = (Kalfa*alfa + (-Kbeta)*(beta + theta_d));
    double UVp = direcao*Kr*rr;
    // Transforma para W do robô
    Mat Ww = Vdiff( UVp, UVp, UWp);
    return Ww;
} 


/* Controle2( deltax, deltay, theta) calculad o valor das velocidades de controle 
    Wd, e We, baseado no artigo de Jong Jin Park (usa Vdiff e Vdiff_p). 
    obs.: k1, k2 > 0 e "r/v" indica o tempo mínimo para chegar ao alvo             */
Mat Controle2(const double& deltax, const double& deltay, const double& theta, 
   const double& theta_d, const double& V, const double& k1, const double& k2, 
	                                                 const double& direcao){
    // Escreve as equações na forma "polar"
    double rr = sqrt(deltax*deltax + deltay*deltay);                      
    double delta = -standardRad((atan2(-deltay, -deltax)) - standardRad(theta));
    double Theta = standardRad(-standardRad(theta) + delta + theta_d);
    double UVp = V;
//    if(direcao == -1){
//	UVp = -V;
//	double theta_d2 = standardRad(theta_d + pi);
//	delta = -standardRad((atan2(-deltay,-deltax)) - standardRad(theta)) - pi;
//	Theta = standardRad(-standardRad(theta) + delta + theta_d2  - pi);
//   }
    double c = atan(-k1*Theta);
    // Calcula os sinais de controle baseado na forma "polar"
    double UWp = -(V/rr)*(k2*(delta - c) + (1 + (k1/(1 +
                       (k1*Theta)*(k1*Theta))))*sin(delta));
    // Transforma para W do robô
    Mat Ww = Vdiff( UVp, UVp, UWp);
    return Ww;
} 


/* Kalman( [x;y;theta], Wd, We, mi_a, Sigma_a) implementa um filtro Kalman esten-
    dido baseado no modelo de um robô diferencial tendo como entradas as imforma-
    ções de odometria. No filtro é feita a fusão entre os dados visuais e  odome-
    tria                                                                           */
Mat Kalman(const Mat& Z, const Mat& PTag, const double& Wd, const double& We, 
					    const Mat& mi_a, const Mat& Sigma_a){
    //Definição de variáveis
    double x_est, y_est, theta_est;
    double g13, g23, k1, k2, k3, k4, v11, v12, v21, v22, v31, v32; 
    // Quantidade dados
    const double m = Z.rows;  
    const double n = Z.cols;
    // Covariância do Sensor Visual
    const double err_x = 0.2;                                                  // (m)
    const double err_y = 0.2;                                                  // (m)
    const double err_tag = 1;                                          
    // Matriz de covariancia
    const Mat Qk = (Mat_<double>(3,3) << err_x*err_x,    0,             0,
                                              0   , err_y*err_y,        0,
                                              0   ,      0     , err_tag*err_tag);
    // Estados iniciais
    const double x_esta = mi_a.at<double>(0,0);
    const double y_esta = mi_a.at<double>(1,0);
    const double theta_esta = mi_a.at<double>(2,0);
    // Deslocamento angular baseado na velocidade
    const double Wd_est = Wd*Ts; 
    const double We_est = We*Ts; 
    // Atualiza o modelo
    const double L = (Wd_est*r + We_est*r)/2;                  	 // Variável auxiliar
    const double Omega = (Wd_est*r - We_est*r)/(2*l);            // Variável auxiliar
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
    // Limita valor de theta_est ao intervalo [-2*pi, 2*pi]
//    if(abs(theta_est)>2*pi)
//	theta_est = (abs(theta_est) - (floor(abs(theta_est)/(2*pi))*2*pi))*(abs(theta_est)/theta_est);
    
    // Calcula a predição da média dos estados
    const Mat mi_barra = (Mat_<double>(3,1) <<   x_est,
		                                 y_est,
		                               theta_est); 
    // Considera singularidades no cálculo das matrizes de Predição
    if(Omega!=0){
        g13 = (L/Omega)*(cos(theta_esta + Omega) - cos(theta_esta));
        g23 = (L/Omega)*(sin(theta_esta + Omega) - sin(theta_esta));
    
        k1 = (r*(Wd_est + We_est))/((2*l)*(Wd_est - We_est));
        k2 = theta_esta + r*((Wd_est - We_est)/(2*l));
        k3 = ((2*l)*We_est)/(2*(r*(Wd_est - We_est)/(2*l))*(r*(Wd_est - We_est)/(2*l)));
        k4 = ((2*l)*Wd_est)/(2*(r*(Wd_est - We_est)/(2*l))*(r*(Wd_est - We_est)/(2*l)));
        v11 = k1*cos(k2) - k3*(sin(k2) - sin(theta_esta));
        v12 = -k1*cos(k2) + k4*(sin(k2) - sin(theta_esta));
        v21 = k1*sin(k2) - k3*(-cos(k2) + cos(theta_esta));
        v22 = -k1*sin(k2) + k4*(-cos(k2) + cos(theta_esta));
        v31 = r/(2*l);
        v32 = -r/(2*l);
    }else{
        g13 = -(L)*sin(theta_esta);
        g23 = (L)*cos(theta_esta);
    
        v11 = r*cos(theta_esta)/2;
        v12 = r*cos(theta_esta)/2;
        v21 = r*sin(theta_esta)/2;
        v22 = r*sin(theta_esta)/2;
        v31 = 0;
        v32 = 0;
    }
    // Calcula as matrizes de Predição
    const Mat Gk = (Mat_<double>(3,3) <<  1, 0, g13,
                                          0, 1, g23,
                                          0, 0,  1  );
    const Mat Vk = (Mat_<double>(3,2) << v11,  v12,
                                         v21,  v22,
                                         v31,  v32);
    // Covariância do encoder, considerado proporcional à leitura
    const double alfa1 = 0.002;
    const double alfa2 = 0.002;
    const Mat Mk = (Mat_<double>(2,2) << (alfa1*(Wd_est))*(alfa1*(Wd_est)),             0,
                                                         0,          (alfa2*abs(We_est))*(alfa2*(We_est)));
    // Predição da variância
    const Mat Sigma_barra = (Gk*Sigma_a*Gk.t()) + (Vk*Mk*Vk.t());
    // Sensor visual modelado como contendo um erro gaussiano e varia da mesma
    // forma que o robô em relação à variação dos estados
    Mat KiHi = (Mat_<double>(3,3) <<  1,0,0,
				      0,1,0,
				      0,0,1);	                  // Memória de Ki*Hi
    Mat KiZ = (Mat_<double>(3,1) <<  0,0,0);		    // Memória de Ki*(Z - zi)
    Mat Mem_KiZ = (Mat_<double>(3,1) <<  0,0,0);
    Mat Aux2 = (Mat_<double>(3,1) <<  0,0,0);
    for(int i=0; i<m/3; i++){
	const double deltai_x = PTag.at<double>(i*3,0) - x_est;
	const double deltai_y = PTag.at<double>(i*3 + 1,0) - y_est;
	const double q = deltai_x*deltai_x + deltai_y*deltai_y;
	// Modelo do sensor visual
	Mat zi = (Mat_<double>(3,1) <<                     sqrt(q),
				       (standardRad(atan2(deltai_y, deltai_x))-theta_est),
						   PTag.at<double>(i*3 + 2,0)		   );
	Mat Hi = (Mat_<double>(3,3) <<  -deltai_x/(sqrt(q)), -deltai_y/(sqrt(q)),  0,
	    			            deltai_y/q,         -deltai_x/q,      -1,
                                                0,                   0,            0  );
//	cout<<"\n zi  "<<zi<<endl<<endl;
	// Ganho de Kalman
	Mat Aux = (Hi*Sigma_barra*Hi.t() + Qk);
	Mat Ki = Sigma_barra*Hi.t()*Aux.inv();
	Mat I = (Mat_<double>(3,3) <<  1, 0, 0,
				       0, 1, 0,
				       0, 0, 1  );
	// Somatórios
	Mat Mem_KiHi = KiHi;			    // Artifício para guardar valores
	KiHi = Mem_KiHi*(I - Ki*Hi);
	Z.rowRange(i*3,i*3+3).copyTo(Aux2);
	Mem_KiZ = KiZ;				    // Artifício para guardar valores
	KiZ = Mem_KiZ + Ki*(Aux2 - zi);
    }
    // Atualizações
    const Mat Sigma = (KiHi)*Sigma_barra;
    Mat mi = mi_barra + KiZ;
    // Adequa o ângulo no intervalo [-2*pi, 2*pi]
//    if(abs(mi.at<double>(2,0))>2*pi)
//	mi.at<double>(2,0) = (abs(abs(mi.at<double>(2,0))) - (floor(abs(mi.at<double>(2,0))/(2*pi))*2*pi))*(abs(mi.at<double>(2,0))/(mi.at<double>(2,0)));
    // Cria a matriz de saídas
    Mat Res(3, 4, CV_64F);
    hconcat(mi, Sigma, Res);

    return Res;   
} 

/* Odometria( theta, Wd, We, mi) implementa um filtro Kalman estendido baseado          
    no modelo de um robô diferencial tendo como entradas as imformaçãoes de odome-
    tria                                                                           */
Mat Odometria( const Mat& mi_a, const double& Wd, const double& We,
					                     const Mat& Sigma_a){
      //Definição de variáveis
    double x_est, y_est, theta_est;
    double g13, g23, k1, k2, k3, k4, v11, v12, v21, v22, v31, v32; 
    double m, n;
    // Quantidade dados
    m = 1;         n = 1;
    // Covariância do Sensor Visual
    double err_x = 0.05;                                                       // (m)
    double err_y = 0.05;                                                       // (m)
    double err_theta = 0.05;                                          // 5.1566 graus
    // Matriz de covariancia
    Mat qk = (Mat_<double>(3,3) << err_x*err_x,    0,             0,
                                        0   , err_y*err_y,        0,
                                        0   ,      0   , err_theta*err_theta);
    Mat Imn = Mat::eye( 1, 1, CV_64F);
    Mat Qk = Kron(qk,Imn);
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
        theta_est = (theta_esta + Omega);
    }else{
        x_est = x_esta + L*cos(theta_esta);
        y_est = y_esta + L*sin(theta_esta);
        theta_est = theta_esta;
    }
    if(abs(theta_est)>=2*pi)
	theta_est = (abs(theta_est) - (floor(abs(theta_est)/(2*pi))*2*pi))*(abs(theta_est)/theta_est);
    
    // Calcula a predição da média dos estados
    Mat mi_barra = (Mat_<double>(3,1) <<   x_est,
		                           y_est,
		                         theta_est); 
    // Considera singularidades no cálculo das matrizes de Predição
    if(Omega!=0){
        g13 = (L/Omega)*(cos(theta_esta + Omega) - cos(theta_esta));
        g23 = (L/Omega)*(sin(theta_esta + Omega) - sin(theta_esta));
    
        k1 = (r*(Wd_est + We_est))/((2*l)*(Wd_est - We_est));
        k2 = theta_esta + r*((Wd_est - We_est)/(2*l));
        k3 = ((2*l)*We_est)/(2*(r*(Wd_est - We_est)/(2*l))*(r*(Wd_est - We_est)/(2*l)));
        k4 = ((2*l)*Wd_est)/(2*(r*(Wd_est - We_est)/(2*l))*(r*(Wd_est - We_est)/(2*l)));
        v11 = k1*cos(k2) - k3*(sin(k2) - sin(theta_esta));
        v12 = -k1*cos(k2) + k3*(sin(k2) - sin(theta_esta));
        v21 = k1*sin(k2) - k3*(-cos(k2) + cos(theta_esta));
        v22 = -k1*sin(k2) + k3*(-cos(k2) + cos(theta_esta));
        v31 = r/(2*l);
        v32 = -r/(2*l);
    }else{
        g13 = -(L)*sin(theta_esta);
        g23 = (L)*cos(theta_esta);
    
        v11 = r*cos(theta_esta)/2;
        v12 = r*cos(theta_esta)/2;
        v21 = r*sin(theta_esta)/2;
        v22 = r*sin(theta_esta)/2;
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

    // Cria a matriz de saídas
    Mat Res(3, 4, CV_64F);
    hconcat(mi_barra, Sigma_barra, Res);
    
    return Res;   
} 
