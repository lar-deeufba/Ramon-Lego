/*
 * @file TesteLego.cpp
 * @brief: Teste do controlador do artigo de Jong usando o lego
 * @author: Ramon O. Fernandes
 * @versão 1.0
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
// Funcionalidades do OpenCV
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
// Include da porta serial
#include "serial_port.hpp"
// Inclui as Funções utilizadas
#include "Funcoes.hpp"

using namespace cv;
using namespace std;

/*------------------------------------Constantes-----------------------------------*/
// Declarações da porta serial
char TX_buffer[5];
char RX_buffer[11];
// Define a Posição final desejada
const double pi = 3.14159265358979323846;
const double Xdes = -2;	   const double Ydes = 0;         const double theta_des = pi;
double direcao;
//Ponto Inicial
Mat mi = (Mat_<double>(3,1) <<   0,
                                 0,
                                 0);
/*---------------------------------------------------------------------------------*/
class Demo{
    // Define contantes
    const char* pt_txt1;    
    SerialPort *sp;

    public:    
    Demo() :                                                            // Construtor
        pt_txt1("Dados.txt")
    {}

    /* Setup dos parâmetros                                                        */
    void setup(){
	LimpaHistorico(pt_txt1);  
	// Inicializa a porta serial para comunicação com o Rover
        char *path = "/dev/ttyACM0";
        sp = SerialPort::getInstance();
        sp->setPort(path);
        if(!sp->isConnected())
            sp->connectSerial();
        if(sp->isConnected()){
            std::cout << "Rover conectado" << std::endl;
        }
        else{
            std::cout << "Rover não conectado" << std::endl;
            exit(8);
        }
    }

    /* Envia os sinais de controle para o robô                                     */
    void EnviaU(const double& Ud, const double& Ue){
        // Verifica a conexão
        if(sp->isConnected()){
            // Define o sentido do movimento
            unsigned  char sUd, sUe;
            if (Ud<0)
                sUd = 'b';
            else
                sUd = 'f';
            if (Ue<0)
                sUe = 'b';
            else
                sUe = 'f';
            // Converte de double para char
            unsigned char cUd = abs(Ud);
            unsigned char cUe = abs(Ue);

            // Escreve os valores no buffer (em forma de char)
                TX_buffer[0] = '#';
                TX_buffer[1] = (unsigned char) cUd;
                TX_buffer[2] = (unsigned char) sUd;
                TX_buffer[3] = (unsigned char) cUe;
                TX_buffer[4] = (unsigned char) sUe;
                // Descomente para imprimir na tela
    /*	    std::cout << "\tEnvia: Ud " << Ud << "  Ue " << Ue <<  std::endl;
            std::cout << "\tEnvia: sUd " << sUd << "  sUe " << sUe <<  std::endl;
    */	    sp->sendSerialData(TX_buffer, 5);
        }
        else{
            std::cout << "\n \t Erro ao enviar, conexao nao detectada!" << std::endl;
        }
    }

    /*  Recebe os sinais de velocidade para o robô                                 */
    Mat RecebeU(){
    int read = -1;
    // Limpa a porta serial
        sp->clearSerialData();
        // Leitura de dados
        read = sp->readSerialData(RX_buffer,sizeof RX_buffer);
//        std::cout << "read: " << read << " bytes" << std::endl;
        // Verifica se houve recepção
        if(read==-1){
            std::cout << "\n \t Erro na recepção !" << std::endl;
        }
        // Reordena o pacote recebido
        int t=0, k=0;
        for(int i=0;i<=10;i++){
            if(RX_buffer[i]='@'){
                for(k=0;k<=10;k++){
                    t=k+i;
                    if(t<=10){
                        RX_buffer[k]=RX_buffer[t];
                    }
                    else{
                        RX_buffer[k]=RX_buffer[t-11];
                    }
                }
                break;
            }
        }
        // Para garantir que os valores lidos foram válidos
        if(RX_buffer[0]=='@'){
        }
        else{
          std::cout << "\n \t Erro, pacote de recepção fora de ordem!" << std::endl;
        }
        // Transfere valores e mostra na tela
        Mat Dados = (Mat_<double>(6,1) << 0,0,0,0,0,0);
        double sinalVd = (RX_buffer[1]?0:1);
        if(sinalVd==1)
            sinalVd=1;
        else
            sinalVd=-1;
        double sinalVe = (RX_buffer[4]?0:1);
        if(sinalVe==1)
            sinalVe=1;
        else
            sinalVe=-1;
        double sinalUd = (RX_buffer[9]?0:1);
        if(sinalUd==1)
            sinalUd=1;
        else
            sinalUd=-1;
        double sinalUe = (RX_buffer[10]?0:1);
        if(sinalUe==1)
            sinalUe=1;
        else
            sinalUe=-1;
        Dados.at<double>(0,0) = (sinalVd*((unsigned char) RX_buffer[2])/127)*16.3398;
        Dados.at<double>(1,0) = (sinalVe*((unsigned char) RX_buffer[5])/127)*16.3398;
        Dados.at<double>(2,0) = sinalUd*((unsigned char) RX_buffer[3]);
        Dados.at<double>(3,0) = sinalUe*((unsigned char) RX_buffer[6]);
        Dados.at<double>(4,0) = ((unsigned char) RX_buffer[7]);
        Dados.at<double>(5,0) = ((unsigned char) RX_buffer[8]);
        // Descomente para imprimir na tela
    //	std::cout << "\n\tWd = " << Dados.at<double>(0,0) << "  We = " << Dados.at<double>(1,0) <<
    //		     "\n\tUd = " << Dados.at<double>(2,0) << "  Ue = " << Dados.at<double>(3,0) << std::endl;
        return Dados;
    }

    /* Executa leituras e comandos do robô dentro do loop                          */ 
    void Robo(const double& t, bool& FlagI){        
        // Inicializa variáveis
        double Wdm, Wem;
        double Udm,Uem;
        double k1,k2;
        double RefD, RefE;
        double mi1, mi2, mi3;
        Mat WW, Ek;

        // Verifica o sentido do movimento
        if(FlagI == 1){
            direcao = Direcao(mi.at<double>(0,0) - Xdes,  mi.at<double>(1,0) - Ydes,
                                                                 mi.at<double>(2,0));
            FlagI = 0;
        }
        // Lê a velocidade das rodas
        Mat Dados = RecebeU();
        Wdm = Dados.at<double>(0,0);   		 // Velocidade medida da roda direita
        Wem = Dados.at<double>(1,0);   	     // Velocidade medida da roda esquerda
        Udm = Dados.at<double>(2,0);	     // Controle realmente aplicado pelo Rover
        Uem = Dados.at<double>(3,0);	     // Controle realmente aplicado pelo Rover
        k1 = Dados.at<double>(4,0);		     // Contador de execução do Rover
        k2 = Dados.at<double>(5,0);		     // Contador de envio do Rover

        // Primeira iteração
        cout <<" Mi: "<< mi << endl<< endl;
        Ek = Odometria( mi, Wdm, Wem);
        mi1 = Ek.at<double>(0,0);			    mi.at<double>(0,0) = mi1;
        mi2 = Ek.at<double>(1,0);			    mi.at<double>(1,0) = mi2;
        mi3 = Ek.at<double>(2,0);			    mi.at<double>(2,0) = mi3;
        // Erros de Posição
        double Ex = -Xdes + mi1;
        double Ey = -Ydes + mi2;
        double theta = Ek.at<double>(2,0);
        double delta_theta = -theta_des + theta;

        // Verifica se atingiu a posição desejada
        if (abs(Ex)<0.1 && abs(Ey)<0.1 && abs(delta_theta)<30*pi/180){
            int f = 1;					  // Flag que indica q parou
                std::cout << "\n Atingiu o critério de 0.1"<<std::endl;
            while (f == 1){
            // Para o Robô
            EnviaU(0, 0);
            // Lê a velocidade das rodas
            Dados = RecebeU();
            Wdm = Dados.at<double>(0,0);   	 // Velocidade medida da roda direita
            Wem = Dados.at<double>(1,0);   	// Velocidade medida da roda esquerda
            Udm = Dados.at<double>(2,0);// Controle realmente aplicado pelo Rover
            Uem = Dados.at<double>(3,0);// Controle realmente aplicado pelo Rover
            k1 = Dados.at<double>(4,0);	     // Contador de execução do Rover
            k2 = Dados.at<double>(5,0);	        // Contador de envio do Rover

            // Atualiza estados
            Ek = Odometria( mi, Wdm, Wem);
            mi1 = Ek.at<double>(0,0);		    mi.at<double>(0,0) = mi1;
            mi2 = Ek.at<double>(1,0);		    mi.at<double>(1,0) = mi2;
            mi3 = Ek.at<double>(2,0);		    mi.at<double>(2,0) = mi3;

            // Cria histórico
            Historico(mi1,mi2,0,0,Wdm,Wem,Udm,Uem,mi3,delta_theta,pt_txt1);
            if (Wdm==0 && Wem==0)
              f = 0;
            }
            std::cout << "\nOperação Finalizada! :)"<<std::endl;
            exit(7);
        }
        else{
            // Escreve na forma polar e retorna o sinal de controle desejado
            WW = Controle2(Ex, Ey, mi3, theta_des, 0.05, 1, 5, direcao);
            double  RefD = WW.at<double>(0,0);
            double  RefE = WW.at<double>(1,0);
            // Envia os comandos
            EnviaU(RefD, RefE);
            // Cria histórico
            Historico(mi1,mi2,RefD,RefE,Wdm,Wem,Ex,Ey,mi3,delta_theta,pt_txt1);
        }
    }
    
    /*  Loop                                                                       */
    void loop(){
        bool FlagI = 1;
	double t0 = tic();
	while (true){    
	    double t = t0 - tic();
	    Robo(t, FlagI);
	    if (cv::waitKey(1) >= 0) 
                break;
       }
    }
}; // Demo


int main(int argc, char* argv[]){
    Demo demo;
    // Setup
    demo.setup();
    // Onde as coisas acontecem
    demo.loop();
}



