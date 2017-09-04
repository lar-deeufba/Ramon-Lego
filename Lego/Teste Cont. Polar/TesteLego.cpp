/*
 * @file TesteLego.cpp
 * @brief Teste do controlador polar para o Lego.
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
const double pi = 3.14159265358979323846;
// Declarações da porta serial
char TX_buffer[5];
char RX_buffer[11];
// Define a Posição final desejada
double Xdes = 0;		double Ydes = 1;		double theta_des = pi;
//Ponto Inicial
Mat mi = (Mat_<double>(3,1) <<   1,
	                         0,
	                         pi); 
double direcao;
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
            std::cout << "Arduíno conectado" << std::endl;
	}
        else{
            std::cout << "Arduíno não conectado" << std::endl;
	    exit(8);
	}
    }

    /* Envia os sinais de controle para o robô                                     */
    void EnviaU(const double& U1, const double& U2){
        // Verifica a conexão
        if(sp->isConnected()){   
	    // Define o sentido do movimento
	    unsigned  char sU1, sU2;
	    if (U1<0)
		sU1 = 'b';
            else
		sU1 = 'f';
	    if (U2<0)
		sU2 = 'b';
            else
		sU2 = 'f';
            // Converte de double para char
	    double absU1 = abs(U1);
   	    double absU2 = abs(U2);
            unsigned char cU1 = absU1;
            unsigned char cU2 = absU2;
            
	    // Escreve os valores no buffer (em forma de char)
            TX_buffer[0] = '#';
            TX_buffer[1] = (unsigned char) cU1;
            TX_buffer[2] = (unsigned char) sU1;
            TX_buffer[3] = (unsigned char) cU2;
            TX_buffer[4] = (unsigned char) sU2;
            // Descomente para imprimir na tela
//	    std::cout << "\tEnvia: U1 " <<absU1 << "  U2 " <<absU2<<  std::endl;
//   	    std::cout << "\tEnvia: sU1 " <<sU1 << "  sU2 " <<sU2<<  std::endl;

//	    std::cout << "\twrite returned: " << sp->sendSerialData(TX_buffer, 5) << std::endl;
	    sp->sendSerialData(TX_buffer, 5);	  
	}
        else{
            std::cout << "Não conectado" << std::endl;
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
            std::cout << "\tERROR READING!" << std::endl;
        }
        // Para garantir que os valores lidos foram válidos
        if(RX_buffer[0]=='@'){	      
	}
	else{
	  std::cout << "\tERROR!" << std::endl;
	}
	// Transfere valores e mostra na tela
	Mat Dados = (Mat_<double>(6,1) << 0,0,0,0,0,0);
	float sinalMd = (RX_buffer[1]?-1:1);
	float sinalMe = (RX_buffer[4]?-1:1);
	float sinalMud = (RX_buffer[9]?-1:1);
	float sinalMue = (RX_buffer[10]?-1:1);
	Dados.at<double>(0,0) = (sinalMd*((unsigned char) RX_buffer[2])/127)*17.3398;
        Dados.at<double>(1,0) = (sinalMe*((unsigned char) RX_buffer[5])/127)*17.3398;
	Dados.at<double>(2,0) = sinalMud*((unsigned char) RX_buffer[3]);
        Dados.at<double>(3,0) = sinalMue*((unsigned char) RX_buffer[6]);
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
	Wem = Dados.at<double>(1,0);   	        // Velocidade medida da roda esquerda
	Udm = Dados.at<double>(2,0);	    // Controle realmente aplicado pelo Rover
	Uem = Dados.at<double>(3,0);	    // Controle realmente aplicado pelo Rover
	k1 = Dados.at<double>(4,0);		     // Contador de execução do Rover
	k2 = Dados.at<double>(5,0);		        // Contador de envio do Rover
	
	// Primeira iteração
	Ek = Odometria( mi, Wdm, Wem);
	mi1 = Ek.at<double>(0,0);			    mi.at<double>(0,0) = mi1;
	mi2 = Ek.at<double>(1,0);			    mi.at<double>(1,0) = mi2;
	mi3 = Ek.at<double>(2,0);			    mi.at<double>(2,0) = mi3;
	// Erros de Posição
	double Ex = mi1 - Xdes;       
	double Ey = mi2 - Ydes;
	double theta = Ek.at<double>(2,0);
	double delta_theta = theta - theta_des;
	
	// Verifica se atingiu a posição desejada
	if (abs(Ex)<0.1 && abs(Ey)<0.1 && abs(delta_theta)<15*pi/180){
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
		Historico(mi1,mi2,0,0,Wdm,Wem,Udm,Uem,mi3,k2,pt_txt1);	
		if (Wdm==0 && Wem==0)
		  f = 0;
	    }
	    std::cout << "\n Operação Finalizada! :)"<<std::endl;
	    exit(7);
	}
	else{
	    // Escreve na forma polar e retorna o sinal de controle desejado
	    WW = Controle(Ex, Ey, theta, theta_des, direcao);
	    double  RefD = WW.at<double>(0,0);
	    double  RefE = WW.at<double>(1,0);
	    // Envia os comandos
	    EnviaU(RefD, RefE);
	    // Cria histórico 
	    Historico(mi1,mi2,RefD,RefE,Wdm,Wem,Udm,Uem,mi3,k2,pt_txt1);	
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



