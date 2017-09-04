/*
  * @file teste.cpp
 * @brief PVBS para o Rover usando Apriltags, OpenCV.
 * @author: Ramon O. Fernandes
 * @versão 3.1(filtro de Kalman, modelo corrigido, Controle Jong Jin, múltiplos Tags)
 *
 * Abre a câmera usb (segunda câmera no sistema) e detecta a AprilTag indicada, e cal-
 * cula a postura relativa do robô. As imagens e as detecções são mostradas em uma ja-
 * nela, e as estimações de postura no prompt. Essa funcionalidade foi criada baseado
 * no algorítimo exemplo feito por Michael Kaess.
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
// Família do Aptiltags e o detector
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag16h5.h"
#include "AprilTags/Tag25h7.h"
#include "AprilTags/Tag25h9.h"
#include "AprilTags/Tag36h9.h"
#include "AprilTags/Tag36h11.h"

// Funcionalidades do OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui_c.h>

// Include da porta serial
#include "serial_port.hpp"
// Inclui as Funções utilizadas
#include "Funcoes.hpp"

using namespace cv;
using namespace std;

/*------------------------------------Constantes-----------------------------------*/
/*  Para evitar repetições, duplicidades e uma boa inicialização,  é definida uma 
    funções para iniciar as constantes globais.                                    */

const double pi = 3.14159265358979323846;
const double Uo = 320;				      // Ponto principal na direção u
const double& Vo = 240;				      // Ponto principal na direção v
const double& Nu = 640;				 // Quantidade de pixels na direção u
const double& Nv = 480;				 // Quantidade de pixels na direção v		   
const double& F = 728.6;				       // Distância focal (m)
const double& Fy = 728.6;				       // Distância focal (m)

// Variáveis declaradas como global para poder acessar em outras funções
Eigen::Vector3d translation;
Mat transl = (Mat_<double>(3,1) << 0,0,0);
Mat Ek;						      // Resposta do filtro de Kalman
Mat Sigma = Mat::zeros(3, 3, CV_64F);		 // Desvio do estado filtro de Kalman
Mat mi = Mat::zeros(3, 1, CV_64F);	       // Média do estado do filtro de Kalman
double yaw, pitch, roll;
bool f_detection = 0;				 // Flag que indica a detecção do tag
char TX_buffer[5];				       // Declaraçãos da porta serial
char RX_buffer[9];				       // Declaraçãos da porta serial
int tt = 0;
// Define a Posição final desejada
double Xdes = 0.75;      	double Ydes = 0;		double theta_des = pi;
double direcao;
// Janela do OpenCV
const char* windowName = "PVBS Rover com AprilTags";

/*---------------------------------------------------------------------------------*/
class Demo{
    // Classe de AprilTags	
    AprilTags::TagDetector* m_tagDetector;
    AprilTags::TagCodes m_tagCodes;
    // Classe de video do OpenCV
    cv::VideoCapture m_cap;
    bool m_draw; // draw image and April tag detections?
    // Define contantes
    int m_width; 
    int m_height;
    double m_tagSize;        // April tag side length in meters of square black frame
    double m_fx;
    double m_fy;
    double m_px; 
    double m_py;
    const char* pt_txt;    
    const char* pt_txt1;    
    const char* pt_txt2;    
    
    int m_deviceId; 

    list<string> m_imgNames;

    SerialPort *sp;

    public:    
    Demo() :                                                            // Construtor
        // Setup, inicializa contantes
        // Mudar linha abaixo para selecionar a família do Tag
        m_tagDetector(NULL),
        m_tagCodes(AprilTags::tagCodes36h11),
        m_draw (false),			     // Imprimir imagem e detecções do April?
        m_width (Nu), 
        m_height (Nv),
        m_tagSize (0.146),                                // Tamanho do Tag em metros
        m_fx (F),
        m_fy (Fy),
        m_px (Uo), 
        m_py (Vo),
        pt_txt("Dadosk.txt"),
        pt_txt1("Dados.txt"),
        pt_txt2("DadosSemI.txt"),
	m_deviceId(1)
    {}
    
    /* Setup dos parâmetros                                                        */
    void setup(){
        m_tagDetector = new AprilTags::TagDetector(m_tagCodes);
        //Janela para mostrar as imagens
        // cv::namedWindow(windowName, 1);
        //Reseta os históricos
        LimpaHistorico(pt_txt);  
	LimpaHistorico(pt_txt1);  
	LimpaHistorico(pt_txt2);  

	// Inicializa a porta serial para comunicação com o Rover
        char *path = "/dev/ttyUSB0";
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

    /* Setup dos parâmetros do video                                               */
    void setupVideo(){
        // Encontra a câmera USB 
        m_cap = cv::VideoCapture(m_deviceId);
        if(!m_cap.isOpened()){
            cerr << "ERRO: Impossível conectar à câmera \n";
            exit(1);
        }
        m_cap.set(CV_CAP_PROP_FRAME_WIDTH, m_width);
        m_cap.set(CV_CAP_PROP_FRAME_HEIGHT, m_height);
        cout << "Camera conectada" << endl;
        cout << "Resolução "<< m_cap.get(CV_CAP_PROP_FRAME_WIDTH) << "x"
             << m_cap.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;
    }   

    /* Estimação de posição do tag                                                 */
    Mat print_detection(AprilTags::TagDetection& detection) const{
        // Posição relativa do Tag:
        Eigen::Matrix3d rotation;
        detection.getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py,
                                                            translation, rotation);
	Eigen::Matrix3d F;
        F <<
             1,  0,  0,
             0,  1,  0,
             0,  0,  1;
        Eigen::Matrix3d rot = F*rotation;
        Mat f_rot = (Mat_<double>(3,3) << rot(0,0), rot(0,1), rot(0,2),           
                                          rot(1,0), rot(1,1), rot(1,2),   
                                          rot(2,0), rot(2,1), rot(2,2));

        wRo_to_euler(f_rot, yaw, pitch, roll);
	double dist = translation.norm();                                           
	// Descomente para imprimir na tela
/*          cout << "  distance=" << translation.norm()
             << "m, x=" << translation(0)
             << ", y=" << translation(1)
             << ", z=" << translation(2)
             << ", yaw=" << yaw
             << ", pitch=" << pitch
             << ", roll=" << roll
             << endl;
*/	
        f_detection = 1;
	Mat Out = (Mat_<double>(3,1) << translation(0), translation(1), yaw);

    }
   
    /* Realiza a translação nas estimações do Tag de acordo com sua Posição        */
    void CompatibilizaCoordenadas(cv::Mat& Dados,const int& i, const int& id) 
    {    
	Mat H, DadosVelho, DadosNovo;
        switch (id){
	    case 1:
		H = (Mat_<double>(3,3) << 1,  0,  0, 
		                          0,  1,  0,
		                          0,  0,  1);
		);
		break;
	    case 2:
		H = (Mat_<double>(3,3) << 1,  0,   0, 
		                          0,  1,  0.2,
		                          0,  0,   1  );
		break;
	    case 3:
		H = (Mat_<double>(3,3) << 1,  0,   0, 
		                          0,  1, -0.2,
		                          0,  0,   1  );
		break;
	    default:
                cout << "\nErro Tag com id Inválido " << endl;
		break;
        }
        
        DadosVelho = (Mat_<double>(3,1) << Dados.at<double>(i,0),
					   Dados.at<double>(i,1),
					             1           );
        DadosNovo = H*DadosVelho;
      	Dados.at<double>(i,0) = DadosNovo.at<double>(1,0);
	Dados.at<double>(i,1) = DadosNovo.at<double>(2,0);
    }

    /* Processa imagem                                                             */
    void processImage(cv::Mat& I, cv::Mat& Ibw) 
    {
        Mat DadosTag, Coluna, Medias;
        // Passa a imagem para escala de cinza 
        cv::cvtColor(I, Ibw, CV_RGB2GRAY);
        double t0 = tic();
        // Detecta AprilTags 
	vector<AprilTags::TagDetection> detections = m_tagDetector->extractTags(Ibw);
        double dt = tic()-t0;
        for (int i=0; i<detections.size(); i++){
             Mat row = print_detection(detections[i]);
             cout << "\nProcessamento demorou " << dt << " s" << endl;
	     DadosTag.push_back(row);                    
        }  
        // Função que translada as detecções
        for (int i=0; i<detections.size(); i++){
            CompatibilizaCoordenadas( DadosTag, i, detection.id);
        }
        //Descomente para mostrar as imagens com o tag realçado
//        for (int i=0; i<detections.size(); i++){
//            detections[i].draw(I);
//        }
//        imshow(windowName, I); // OpenCV call


//	  Eigen::Matrix3d M;
//  M <<
//    0, 0, 1,
//    1, 0, 0,
//    0, 1, 0;
  //Eigen::Matrix4d MT = M*T;
  // translation vector from camera to the April tag
//  trans = T.col(3).head(3);
  // orientation of April tag with respect to camera: the camera
  // convention makes more sense here, because yaw,pitch,roll then
  // naturally agree with the orientation of the object
//  rot = T.block(0,0,3,3);
  // reverte a rotação
// trans = -rot.transpose()*trans;
//  trans = M*trans;
//  rot = M*rot;


	// Tira a média das estimações
        for (int i=0; i == 2; i++){
	     
	    transl.at<double>(i,0) = mean(DadosTag.col(i));
	  
	} 
	cout << "m, x=" << transl.at<double>(1,0)
             << ", y=" << transl.at<double>(2,0)
             << ", teta=" << transl.at<double>(3,0)
             << endl;	
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
/*	    std::cout << "\tEnvia: U1 " <<absU1 << "  U2 " <<absU2<<  std::endl;
   	    std::cout << "\tEnvia: sU1 " <<sU1 << "  sU2 " <<sU2<<  std::endl;
*/
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
        else if(read == 5){
//            std::cout << "Leu" << std::endl;
        }
        // Para garantir que os valores lidos foram válidos
        if(RX_buffer[0]=='@'){	      
	}
	else{
	  std::cout << "\tERROR!" << std::endl;
	}
	// Transfere valores e mostra na tela
	Mat Dados = (Mat_<double>(6,1) << 0,0,0,0,0,0);
	int sinalM1 = (RX_buffer[1]?-1:1);
	int sinalM2 = (RX_buffer[4]?-1:1);
        Dados.at<double>(0,0) = sinalM1*((unsigned char) RX_buffer[2]);
        Dados.at<double>(1,0) = sinalM2*((unsigned char) RX_buffer[5]);
	Dados.at<double>(2,0) = sinalM1*((unsigned char) RX_buffer[3]);
        Dados.at<double>(3,0) = sinalM2*((unsigned char) RX_buffer[6]);
	Dados.at<double>(4,0) = ((unsigned char) RX_buffer[7]);
	Dados.at<double>(5,0) = ((unsigned char) RX_buffer[8]);
	// Descomente para imprimir na tela
//	std::cout << "\n\tWd = " << Dados.at<double>(0,0) << "  We = " << Dados.at<double>(1,0) << 
//		     "\n\tUd = " << Dados.at<double>(2,0) << "  Ue = " << Dados.at<double>(3,0) << std::endl;
	return Dados;
    }

    /* Executa leituras e comandos do robô caso o tag seja detectado               */ 
    void Robo_Tag(){        
	// Inicializa variáveis
	double Wdm, Wem;
	double Udm,Uem;
	double k1,k2;
	double Xest, Yest, theta_est;
	double Ex, Ey, delta_theta;
	double mi1, mi2, mi3, s1, s2, s3, s4, s5, s6, s7, s8 ,s9;
	// Reseta o flag da detecção
        f_detection = 0;  
        
	// Estimação de posição
        Xest = transl.at<double>(0,0);
	Yest = transl.at<double>(1,0); 
	theta_est = pi/2 + transl.at<double>(2,0); 

	// Lê a velocidade das rodas
	Mat Dados = RecebeU();
	Wdm = Dados.at<double>(0,0);   		 // Velocidade medida da roda direita
	Wem = Dados.at<double>(1,0);   		// Velocidade medida da roda esquerda
	Udm = Dados.at<double>(2,0);	    // Controle realmente aplicado pelo Rover
	Uem = Dados.at<double>(3,0);	    // Controle realmente aplicado pelo Rover
	k1 = Dados.at<double>(4,0);		     // Contador de execução do Rover
	k2 = Dados.at<double>(5,0);		        // Contador de envio do Rover
	
	// Primeira iteração do filtro
	if(tt==0){
	    Ek = (Mat_<double>(3,1) << Xest, Yest, theta_est); 	
	    tt = 1;
	  
	}
	else{
	    Ek = Kalman( Xest, Yest, theta_est, Wdm, Wem, mi, Sigma);
	}
	mi1 = Ek.at<double>(0,0);			  mi.at<double>(0,0) = mi1;
        mi2 = Ek.at<double>(1,0);			  mi.at<double>(1,0) = mi2;
        mi3 = Ek.at<double>(2,0);			  mi.at<double>(2,0) = mi3;
	
	s1 = Ek.at<double>(0,1);			Sigma.at<double>(0,0) = s1;
	s2 = Ek.at<double>(1,1);			Sigma.at<double>(1,0) = s2;
	s3 = Ek.at<double>(2,1);			Sigma.at<double>(2,0) = s3;
	s4 = Ek.at<double>(0,2);			Sigma.at<double>(0,1) = s4;
	s5 = Ek.at<double>(1,2);			Sigma.at<double>(1,1) = s5;
	s6 = Ek.at<double>(2,2);			Sigma.at<double>(2,1) = s6;
	s7 = Ek.at<double>(0,3);			Sigma.at<double>(0,2) = s7;
	s8 = Ek.at<double>(1,3);			Sigma.at<double>(1,2) = s8;
	s9 = Ek.at<double>(2,3);			Sigma.at<double>(2,2) = s9;
	
	// Erros de Posição
	Ex = mi1 - Xdes;       
	Ey = mi2 - Ydes;
	delta_theta = mi3 - theta_des;

	// Cria histórico 
	Historico( Ek.at<double>(0,0),Ek.at<double>(1,0),Ek.at<double>(2,0), Ex, Ey,
							  delta_theta,0,0,0,0,pt_txt);	
	// Descomente para imprimir a estimação de posição na tela
	//std::cout << "\nXest: "<<Xest<<"      Yest:"<<Yest<<std::endl;
	// Verifica se atingiu a posição desejada
	if (abs(Ex)<0.05 && abs(Ey)<0.05 && abs(delta_theta)<0.05){
	    int f = 1;					  // Flag que indica q parou 
	    while (f == 1){
		// Para o Robô
		EnviaU(0, 0);
		// Lê a velocidade das rodas
		Mat Dados = RecebeU();
		Wdm = Dados.at<double>(0,0);   	 // Velocidade medida da roda direita
		Wem = Dados.at<double>(1,0);   	// Velocidade medida da roda esquerda
		Udm = Dados.at<double>(2,0);// Controle realmente aplicado pelo Rover
		Uem = Dados.at<double>(3,0);// Controle realmente aplicado pelo Rover
		k1 = Dados.at<double>(4,0);	     // Contador de execução do Rover
		k2 = Dados.at<double>(5,0);	        // Contador de envio do Rover
		// Cria histórico  
		Historico( Ek.at<double>(0,0),Ek.at<double>(1,0),Ek.at<double>(2,0), Ex, Ey,
							   delta_theta,0,0,0,0,pt_txt);	
		Historico(Xest,Yest,0,0,Wdm,Wem,Udm,Uem,theta_est,k1,pt_txt1);	
		if (Wdm==0 && Wem==0)
		  f = 0;
	    }
	    std::cout << "\nOperação Finalizada! :)"<<std::endl;
	    exit(7);
	}
	else{
	    // Escreve na forma polar e retorna o sinal de controle desejado
	    Mat WW = Controle2( Ex, Ey, mi3, 2, 1, 3);
	    double  RefD = WW.at<double>(0,0);
	    double  RefE = WW.at<double>(1,0);
	    // Envia os comandos
	    EnviaU(RefD, RefE);
	    // Cria histórico 
	    Historico(Xest,Yest,RefD,RefE,Wdm,Wem,Udm,Uem,theta_est,k1,pt_txt1);	
	}
    }

    /* Executa leituras e comandos do robô caso o tag não seja detectado           */ 
    void Robo_Odometria(){        
	// Inicializa variáveis
	double Wdm, Wem;
	double Udm,Uem;
	double k1, k2;
	double mi1, mi2, mi3;
	
	// Lê a velocidade das rodas
	Mat Dados = RecebeU();
	Wdm = Dados.at<double>(0,0);   		 // Velocidade medida da roda direita
	Wem = Dados.at<double>(1,0);   		// Velocidade medida da roda esquerda
	Udm = Dados.at<double>(2,0);	    // Controle realmente aplicado pelo Rover
	Uem = Dados.at<double>(3,0);	    // Controle realmente aplicado pelo Rover
	k1 = Dados.at<double>(4,0);		     // Contador de execução do Rover
	k2 = Dados.at<double>(5,0);		        // Contador de envio do Rover
	
	Ek = Odometria( mi, Wdm, Wem);
	mi1 = Ek.at<double>(0,0);			    mi.at<double>(0,0) = mi1;
        mi2 = Ek.at<double>(1,0);			    mi.at<double>(1,0) = mi2;
        mi3 = Ek.at<double>(2,0);			    mi.at<double>(2,0) = mi3;
	
	// Erros de Posição
	double Ex = mi1 - Xdes;       
	double Ey = mi2 - Ydes;
        double theta = Ek.at<double>(2,0);
	// Cria histórico 
	Historico(0,0,0,0,0,0,0,0,0,0,pt_txt);	
	// Verifica se atingiu a posição desejada
	if (abs(Ex)<0.05 && abs(Ey)<0.05){
	    int f = 1;					  // Flag que indica q parou 
	    while (f == 1){
		// Para o Robô
		EnviaU(0, 0);
		// Lê a velocidade das rodas
		Mat Dados = RecebeU();
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
		Historico(mi1,mi2,0,0,Wdm,Wem,Udm,Uem,mi3,k1,pt_txt2);	
		if (Wdm==0 && Wem==0)
		  f = 0;
	    }
	    std::cout << "\nOperação Finalizada! :)"<<std::endl;
	    exit(7);
	}
	else{
	    // Escreve na forma polar e retorna o sinal de controle desejado
	    Mat WW = Controle2( Ex, Ey, mi3, 3, 1, 10);
	    double  RefD = WW.at<double>(0,0);
	    double  RefE = WW.at<double>(1,0);
	    // Envia os comandos
	    EnviaU(RefD, RefE);
	    std::cout << "\nK"<< k2 <<std::endl;
	    // Cria histórico 
	    Historico(mi1,mi2,RefD,RefE,Wdm,Wem,Udm,Uem,mi3,k1,pt_txt2);	
	}
    }
   
    /*  Loop onde a imagem é adquirida, e o Tag detectado                          */
    void loop(){
        // Matrizes com as imagens
        cv::Mat I;
        cv::Mat Ibw;
	
	int frame = 0;                                        // Para cálculo dos fps
        double last_t = tic();                        // Para medir tempo de execução
        while (true){    
	        // Captura frame
	        m_cap >> I;
	        processImage(I, Ibw);
	        frame++;
	        if (frame % 10 == 0){
                    double t = tic();                 // Para medir tempo de execução
                    cout << "   " << 10./(t-last_t) << " fps" << endl;
                    last_t = t;
                }
                // Caso o alvo seja detectado
	        if (f_detection == 1){
		    double t0 = tic();                // Para medir tempo de execução
		    Robo_Tag();
	            double t = tic() - t0;
	            std::cout << "\nTempo robo " << t <<" s"<< std::endl; 
	    	}
	    	// Caso o alvo não seja detectado usa Kalman e a odometria
	    	else{
	            std::cout << "\nNenhum tag detectado " << std::endl; 
		    //Robo_Odometria();		
		    EnviaU(0,0);
	    	}
            // Finaliza ao pressionar uma tecla
	    if (cv::waitKey(1) >= 0) 
                break;
       }
    }
}; // Demo


int main(int argc, char* argv[]){
    Demo demo;
    // Setup
    demo.setup();
    demo.setupVideo();
    // Onde as coisas acontecem
    demo.loop();
}



