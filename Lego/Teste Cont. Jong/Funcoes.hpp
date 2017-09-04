#ifndef FUNCOES_HPP
#define FUNCOES_HPP

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
// Include da porta serial
#include "serial_port.hpp"

using namespace cv;
using namespace std;

// Declaração das funções
    Mat V2Winv();
    Mat ConstPol();
    double tic();
    inline double standardRad(double t);
    void wRo_to_euler(const Mat& wRo, double& yaw, double& pitch, double& roll);
    void Historico(double arg1, double arg2, double arg3, double arg4, double arg5, 
		  double arg6, double arg7, double arg8, double arg9, double arg10,
		        	    			          const char* name);
    void LimpaHistorico(const char* name);
    Mat Vdiff(const double& Vx, const double& Vy, const double& Wz);
    double Direcao(const double& deltax, const double& deltay, const double& theta);
    Mat Vdiff_p(const double& deltax, const double& deltay, const double& theta);
    Mat Controle(const double& deltax, const double& deltay, const double& theta,
                                      const double& theta_d, const double& direcao); 
    Mat Controle2(const double& deltax, const double& deltay, const double& theta, 
                         const double& theta_d, const double& v, const double& k1, 
		                           const double& k2, const double& direcao);
    Mat Kalman(const double& X, const double& Y, const double& theta, 
           const double& Wd, const double& We, const Mat& mi_a, const Mat& Sigma_a);
    Mat Odometria(const Mat& mi_a, const double& Wd, const double& We);

#endif // FUNCOES_HPP
