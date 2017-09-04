/*
 * @file teste.cpp
 * @brief PVBS para o Rover usando Apriltags, OpenCV.
 * @author: Ramon O. Fernandes
 * @versão 3.3(filtro de Kalman, modelo corrigido, Controle Jong Jin, múltiplos Tags,
 * 						   modelo do alvo de acordo com Thrun)
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
#include <opencv2/imgproc/imgproc.hpp>
#include "highgui.h"


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
const double& F = 755.8;				    // Distância focal pixels
const double& Fy = 755.8;				    // Distância focal pixels
// Variáveis declaradas como global para poder acessar em outras funções
Eigen::Vector3d translation;
Mat transl = (Mat_<double>(3,1) << 0,0,0);
Mat Ek;						      // Resposta do filtro de Kalman
Mat Sigma = Mat::zeros(3, 3, CV_64F);		 // Desvio do estado filtro de Kalman
Mat mi = (Mat_<double>(3,1) << 2,0,pi);        // Média do estado do filtro de Kalman
double yaw, pitch, roll;
bool f_detection = 0;				 // Flag que indica a detecção do tag
char TX_buffer[5];				       // Declaraçãos da porta serial
char RX_buffer[11];				       // Declaraçãos da porta serial
int tt = 0;
bool FlagI = 1;
Mat PTagi;			     // Posição e orientação do Tag "i" indentificado
// Define a Posição final desejada
double Xdes = 1;      	double Ydes = 0.5;		double theta_des = pi/2;
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
    bool m_draw; // draw image and April tag detections
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
        m_draw (false),			      // Imprimir imagem e detecções do April
        m_width (Nu), 
        m_height (Nv),
        m_tagSize (0.146),                                // Tamanho do Tag em metros
        m_fx (F),
        m_fy (Fy),
        m_px (Uo), 
        m_py (Vo),
        pt_txt("Dados.txt"),
        pt_txt1("Dadosk.txt"),
        pt_txt2("DadosZ.txt"),
	m_deviceId(0)
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
	// Inicializa a porta serial para comunicação com o Arduino
        char *path = "/dev/ttyACM0";
        sp = SerialPort::getInstance();
	sp->setPort(path);
	if(!sp->isConnected())
		sp->connectSerial();
        if(sp->isConnected()){
            std::cout << "Arduino conectado" << std::endl;
	}
        else{
            std::cout << "Arduino não conectado" << std::endl;
	    exit(0);	  
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
	m_cap.set(CV_CAP_PROP_BRIGHTNESS , 0.05);
	m_cap.set(CV_CAP_PROP_CONTRAST, 0.5);
	m_cap.set(CV_CAP_PROP_SATURATION , 0.5);
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
        F << 1,  0,  0,
             0, -1,  0,
             0,  0,  1;
        Eigen::Matrix3d rot = F*rotation;
        Mat f_rot = (Mat_<double>(3,3) << rot(0,0), rot(0,1), rot(0,2),           
                                          rot(1,0), rot(1,1), rot(1,2),   
                                          rot(2,0), rot(2,1), rot(2,2));

        wRo_to_euler(f_rot, yaw, pitch, roll);
	double dist = translation.norm();                                           
	// Flag que indica que houve detecção
	f_detection = 1;
//	cout<<"\n "<<translation(0)<<"  "<<translation(1)<<"  "<<translation(2)<<endl;
	Mat Out = (Mat_<double>(1,3) << translation(0), translation(1), pitch);
	return Out;
    }
 
    /* Posição e orientação do tag em coordenadas globais                          */
    void PosicaoTag(const int& id){    
        switch (id){
	    case 1:
		PTagi = (Mat_<double>(1,3) << 0.001, 0.001, 0);
		break;
	    case 2:
		PTagi = (Mat_<double>(1,3) << 0, -0.5, 0);
		break;
	    case 3:
		PTagi = (Mat_<double>(1,3) << 0, 0.5, 0);
		break;
	    case 4:
		PTagi = (Mat_<double>(1,3) << 0.75, 2, -pi/2);
		break;
	    case 5:
		PTagi = (Mat_<double>(1,3) << 0, 2, -pi/2);
		break;
	    case 6:
		PTagi = (Mat_<double>(1,3) << 0, 2, -pi/2);
		break;
	    default:
                cout << "\nErro Tag com id Inválido " << endl;
		break;
        }
    }
    
    /* Realiza a translação nas estimações do Tag de acordo com sua Posição        */
    Mat CompatibilizaCoordenadas(const Mat& Dados, const int& i, const int& id){    
	Mat H, DadosVelho, DadosNovo, M, T, R;
	double Rot, Rot_ant;
        switch (id){
	    case 1:
		H = (Mat_<double>(3,3) << 1,  0,  0, 
		                          0, -1,  0,
		                          0,  0,  1 );
		R = (Mat_<double>(3,3) << 1,  0,  0, 
		                          0,  1,  0,
		                          0,  0,  1 );
		Rot = pi;
		break;
	    case 2:
		H = (Mat_<double>(3,3) << 1,  0,   0, 
		                          0, -1, -0.5,
		                          0,  0,   1  );
		R = (Mat_<double>(3,3) << 1,  0,  0, 
		                          0,  1,  0,
		                          0,  0,  1 );
		Rot = pi;
		break;
	    case 3:
		H = (Mat_<double>(3,3) << 1,  0,   0, 
		                          0, -1,  0.5,
		                          0,  0,   1  );
		R = (Mat_<double>(3,3) << 1,  0,  0, 
		                          0,  1,  0,
		                          0,  0,  1 );
		Rot = pi;
		break;
	    case 4:
		H = (Mat_<double>(3,3) << 1,  0, 0.75, 
		                          0,  1,   2,
		                          0,  0,   1   );
		R = (Mat_<double>(3,3) <<  0,  -1,  0, 
		                          -1,  0,  0,
		                           0,  0,  1 );
		Rot = pi/2;
		break;
	    case 5:
		H = (Mat_<double>(3,3) << 1,  0, 0.5, 
		                          0,  1,  2,
		                          0,  0,  1   );
		R = (Mat_<double>(3,3) << 0,  1,  0, 
		                          1,  0,  0,
		                          0,  0,  1 );
		Rot = pi/2;
		break;
	    case 6:
		H = (Mat_<double>(3,3) << 1,  0, 1.5, 
		                          0,  1,  2,
		                          0,  0,  1  );
		R = (Mat_<double>(3,3) << 0, -1,  0, 
		                          1,  0,  0,
		                          0,  0,  1 );
		Rot = pi/2;
		break;
	    default:
                cout << "\nErro Tag com id Inválido " << endl;
		break;
        }
        
        Rot_ant = Dados.at<double>(i,2);
        DadosVelho = (Mat_<double>(3,1) << Dados.at<double>(i,0),
					   Dados.at<double>(i,1),
					             1           );
        DadosNovo = H*R*DadosVelho;
	// Quantidade dados
	Mat out = Mat::zeros(1, 3, CV_64F);
	out.at<double>(0,0) = DadosNovo.at<double>(0,0);
	out.at<double>(0,1) = DadosNovo.at<double>(1,0);
	out.at<double>(0,2) = Rot_ant + Rot;
	// Descomente para escrever informações na tela
//       cout << "\nAlvo " << id <<":"<< endl;
//	cout << "\nx =" << out.at<double>(0,0)
//             << ", y =" << out.at<double>(0,1) 
//             << ", theta =" << out.at<double>(0,2) 
//             << endl;
	return out;
    }

    /* Processa imagem                                                             */
    Mat processImage(cv::Mat& I, cv::Mat& Ibw) {
        Mat DadosTag,DadosTag2;
        // Passa a imagem para escala de cinza 
        cv::cvtColor(I, Ibw, CV_RGB2GRAY);
        // Detecta AprilTags 
	vector<AprilTags::TagDetection> detections = m_tagDetector->extractTags(Ibw);
        for (int i=0; i<detections.size(); i++){
             Mat rec = print_detection(detections[i]);
	     double rTag = sqrt(rec.at<double>(0,0)*rec.at<double>(0,0) + 
	                                   rec.at<double>(1,0)*rec.at<double>(1,0));
	     double fiTag = rec.at<double>(2,0);
	     Mat row = (Mat_<double>(1,3) << rTag, fiTag, detections[i].id);
	     DadosTag.push_back(row); 
     	     DadosTag2.push_back(rec); 
	     // Posição e orientação do Tag
	     PosicaoTag(detections[i].id);
	     DadosTag.push_back(PTagi);
	     if(FlagI == 1){
		// Função que translada as detecções e impõe a rotação do alvo
		for (int i=0; i<detections.size(); i++){
		    Mat Out2 = CompatibilizaCoordenadas( DadosTag2, i, detections[i].id);
		    DadosTag.push_back(Out2);
		}
	     }
	}
        //Descomente para mostrar as imagens com o tag realçado
        //for (int i=0; i<detections.size(); i++){
        //    detections[i].draw(I);
        //}
        //imshow(windowName, I); // OpenCV call
	return DadosTag;
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
        // Para garantir que os valores lidos foram válidos
        if(RX_buffer[0]=='@'){	      
	}
	else{
	  std::cout << "\n \t Erro, pacote de recepção fora de ordem!" << std::endl;
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

    /* Executa leituras e comandos do robô caso o tag seja detectado               */ 
    void Robo_Tag(const Mat& DadosSensoriais, bool FlagI){        
	// Inicializa variáveis
	double Wdm, Wem, Udm, Uem, k1, k2;
	double Xest=0, Yest=0, theta_est=0;
	double rZ=0, fiZ=0;
	double Ex, Ey, delta_theta;
	Mat WW, Ek, Z, PTag, PoseI;
	// Ajusta os dados dos sensor 
	double m = DadosSensoriais.rows;
	if(FlagI == 1){
	   for(int i = 0; i < m/3; i++){
		if(i>0){
		    vconcat(Z, DadosSensoriais.row(i*2).t(), Z);
		    vconcat(PTag, DadosSensoriais.row(i*2 + 1).t(), PTag);
		    vconcat(PoseI, DadosSensoriais.row(i*2 + 2).t(), PTag);
		}
		else{
		    Z = DadosSensoriais.row(i*2).t();
		    PTag = DadosSensoriais.row(i*2+1).t();
    		    PoseI = DadosSensoriais.row(i*2+2).t();
		}
	    }
	    // Calcula a média das estimações
	   /* for(int i=0; i<m/3; i++ ){
		    Xest = PoseI.at<double>(i,0) + Xest;
		    Yest = PoseI.at<double>(i,1) + Yest;
		    theta_est = PoseI.at<double>(i,2) + theta_est;
	    }
	    Xest = 3*Xest/m;
	    Yest = 3*Yest/m;
	    theta_est = 3*theta_est/m;
	    mi.at<double>(0,0) = Xest;
    	    mi.at<double>(1,0) = Yest;
	    mi.at<double>(2,0) = theta_est;
	  */  // Define o sentido do movimento na primeira vez que executa
	    direcao = Direcao(mi.at<double>(0,0) - Xdes,  mi.at<double>(1,0) - Ydes,
	     		                                        mi.at<double>(2,0));
	    FlagI = 0;
	}else{
	    for(int i = 0; i < m/2; i++){
		if(i>0){
		    vconcat(Z, DadosSensoriais.row(i*2).t(), Z);
		    vconcat(PTag, DadosSensoriais.row(i*2 + 1).t(), PTag);
		}
		else{
		    Z = DadosSensoriais.row(i*2).t();
		    PTag = DadosSensoriais.row(i*2+1).t();
		}
	    }
	}
	// Cria arquivo com log para os dados do sensor
	for(int i=0; i<m/3; i++ ){
	    Historico( Z.at<double>(i,0),Z.at<double>(i,1),Z.at<double>(i,2), m/3, 0,
							          0,0,0,0,0,pt_txt2);	
	}
	// Artifício que organ iza o log de Z, sempre que houver "1" na quinta posi-
	// ção do log se estabelece uma janela de leitura
//	Historico( 0,0,0,0,1,0,0,0,0,pt_txt2);
	// Lê a velocidade das rodas
	Mat Dados = RecebeU();
	Wdm = Dados.at<double>(0,0);   		 // Velocidade medida da roda direita
	Wem = Dados.at<double>(1,0);   		// Velocidade medida da roda esquerda
	Udm = Dados.at<double>(2,0);	     // Controle realmente aplicado pelo Lego
	Uem = Dados.at<double>(3,0);	     // Controle realmente aplicado pelo Lego
	k1 = Dados.at<double>(4,0);		      // Contador de execução do Lego
	k2 = Dados.at<double>(5,0);		         // Contador de envio do Lego
	// Filtro de Kalman
	//Ek = Kalman( Z, PTag, Wdm, Wem, mi, Sigma);
	Ek = Odometria( mi, Wdm, Wem, Sigma);
	// Atualiza mi e Sigma
	mi = Ek.col(0);			
	Ek.colRange(1, 4).copyTo(Sigma);
	// Artifício que aguarda a estimação do ângulo no algorítimo Kalman convergir
//	if(tt<=10){
//	    tt = tt + 1;				       // Atualiza o contador
//	    Ek.at<double>(2,0) = theta_est;
//	}
	// Erros de Postura
	Ex = Ek.at<double>(0,0) - Xdes;             Ey = Ek.at<double>(1,0) - Ydes;
	delta_theta = Ek.at<double>(2,0) - theta_des;
	// Cria histórico 
	Historico( Ek.at<double>(0,0),Ek.at<double>(1,0),Ek.at<double>(2,0), Ex, Ey,
							  delta_theta,0,0,0,0,pt_txt);	
	// Descomente para imprimir a estimação de posição na tela
	cout << "\n Postura Estimada Kalman: \n" << mi << endl << endl;
	// Verifica se atingiu a posição desejada
	if (abs(Ex)<0.05 && abs(Ey)<0.05 && standardRad((abs(delta_theta)))<30*pi/180){
	    int f_parou = 1;				   // Flag que indica q parou 
	    while (f_parou == 1){
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
		Historico(Ek.at<double>(0,0),Ek.at<double>(1,0),0,0,Wdm,Wem,Udm,Uem,
		                                       Ek.at<double>(2,0),k1,pt_txt1);
		if (Wdm==0 && Wem==0)
		  f_parou = 0;
	    }
	    std::cout << "\nOperação Finalizada! :)"<<std::endl;
	    exit(3);
	}
	// Caso não se verifique o critério de parada
	else{
	    // Escreve na forma polar e retorna o sinal de controle desejado
	    Mat WW = Controle2(Ex, Ey, Ek.at<double>(2,0), theta_des, 0.05, 1, 5, direcao);
	    double  RefD = WW.at<double>(0,0);
	    double  RefE = WW.at<double>(1,0);
	    // Artifício que aguarda a estimação do ângulo no algorítimo Kalman convergir
//	    if(tt<11){
//		RefD = 0;
//		RefE = 0;
//	    }
	    // Envia os comandos
	    EnviaU(RefD, RefE);
	    // Cria histórico 
	    Historico(Ek.at<double>(0,0),Ek.at<double>(1,0),RefD,RefE,Wdm,Wem,Udm,Uem,
		                                         Ek.at<double>(2,0),k1,pt_txt1);	
	}
	// Reseta a flag da detecção
        f_detection = 0;  
    }

    /* Executa leituras e comandos do robô caso o tag não seja detectado           */ 
    void Robo_Odometria(){        
	// Inicializa variáveis
	double Wdm, Wem;
	double Udm,Uem;
	double k1, k2;
	double Ex, Ey, delta_theta;
	
	// Lê a velocidade das rodas
	Mat Dados = RecebeU();
	Wdm = Dados.at<double>(0,0);   		 // Velocidade medida da roda direita
	Wem = Dados.at<double>(1,0);   		// Velocidade medida da roda esquerda
	Udm = Dados.at<double>(2,0);	    // Controle realmente aplicado pelo Rover
	Uem = Dados.at<double>(3,0);	    // Controle realmente aplicado pelo Rover
	k1 = Dados.at<double>(4,0);		     // Contador de execução do Rover
	k2 = Dados.at<double>(5,0);		        // Contador de envio do Rover
	
	Ek = Odometria( mi, Wdm, Wem, Sigma);
	mi = Ek.col(0);
	
	// Erros de Postura
	Ex = Ek.at<double>(0,0) - Xdes;             Ey = Ek.at<double>(1,0) - Ydes;
	delta_theta = Ek.at<double>(2,0) - theta_des;
	// Cria histórico 
	Historico(0,0,0,0,0,0,0,0,0,0,pt_txt);	
	Historico(0,0,0, 1, 0,0,0,0,0,0,pt_txt2);	
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
		
		// Atualiza estados
		Ek = Odometria( mi, Wdm, Wem, Sigma);
		mi = Ek.col(0);			    
		// Cria histórico  
		Historico(Ek.at<double>(0,0),Ek.at<double>(1,0),0,0,Wdm,Wem,Udm,Uem,
		                                       Ek.at<double>(2,0),k1,pt_txt1);
		if (Wdm==0 && Wem==0)
		  f = 0;
	    }
	    std::cout << "\nOperação Finalizada! :)"<<std::endl;
	    exit(3);
	}
	else{
	    // Escreve na forma polar e retorna o sinal de controle desejado
	    Mat WW = Controle2(Ex, Ey, Ek.at<double>(2,0), theta_des, 0.05, 1, 5, direcao);
	    double  RefD = WW.at<double>(0,0);
	    double  RefE = WW.at<double>(1,0);
	    // Envia os comandos
	    EnviaU(RefD, RefE);
	    // Cria histórico 
	    Historico(Ek.at<double>(0,0),Ek.at<double>(1,0),RefD,RefE,Wdm,Wem,Udm,Uem,
		                                         Ek.at<double>(2,0),k1,pt_txt1);	
	}
    }
   
    /*  Loop onde a imagem é adquirida, e o Tag detectado                          */
    void loop(){
        // Matrizes com as imagens
        cv::Mat I;
        cv::Mat Ibw;
	cv::Mat DadosSensoriais;
	bool FlagI = 1;
	while (true){    
	        // Captura frame
	        m_cap >> I;
	        DadosSensoriais = processImage(I, Ibw);
                // Caso o alvo seja detectado
	        if (f_detection == 1){
		    std::cout << "\n Tag detectado " << std::endl; 
		    Robo_Tag(DadosSensoriais, FlagI);
	    	}
	    	// Caso o alvo não seja detectado usa Kalman e a odometria
	    	else{
	            std::cout << "\n Nenhum tag detectado " << std::endl; 
		    Robo_Odometria();		
		    //EnviaU(0,0);
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



