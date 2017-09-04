#include <Wire.h>
#include<MsTimer2.h>
#define SLAVE_ADDRESS 0x04
// Variáveis de comunicação
uint8_t c[]="DEXTERIN";
byte recebeLego[8] = {0, 0, 0, 0, 0, 0, 0, 0};
byte enviaSerial[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
byte enviaLego[8] = {0, 0, 0, 0, 0, 0, 0, 0};
//// Quantidade de dados no buffer da velocidade
//int n=0;
//// Buffer da velocidade
//double bufVd[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//double bufVe[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//// Velocidades lidas FILTRADA
//double Vdf, Vef;
//int fBufV=0;          // Flag das 5 primeiras amostras do filtro
// Velocidades lidas em %
double vDp=0,s_vDp=0,vEp=0,s_vEp=0;
// Velocidades em rad/s
double vD=0,vE=0;
// Velocidades desejadas
double refD=0,s_refD=0,refE=0,s_refE=0;
int RefD=0;
int RefE=0;
double kpD=3.2,kiD=3.5,kdD=0,kpE=3.2,kiE=3.5,kdE=0;
double erroD=0,erroDa=0,erroDaa=0,erroE=0,erroEa=0,erroEaa=0;
double uD=0,uDa=0,s_uD=0,uE=0,uEa=0,s_uE=0;
double Ud=0,sUd=0,Ue=0,sUe=0;
// Contador para indicar que houve 10 iteraçãoes com o lego
// o que equivale a 300ms (a temporização é feita no lego)
int flag_lego;
long double tt=0;                       // Contador para debug
long double ttt=0;                      // Contador para debug
double Wmax=17.3398;
double T=0.20;                          // Período de amostragem
/* ------------------------- Funções ------------------------- */
// Envio de dados pela conexao serial para o computador
void EnviaComputador(){
  // Carrega o buffer de envio
  enviaSerial[0] = (byte)'@';
  enviaSerial[1] = (byte)(s_vDp);
  enviaSerial[2] = (byte)(abs(vDp));
  enviaSerial[3] = (byte)(abs(uD));
  enviaSerial[4] = (byte)(s_vEp);
  enviaSerial[5] = (byte)(abs(vEp));
  enviaSerial[6] = (byte)(abs(uE));
  enviaSerial[7] = (byte)(tt);
  enviaSerial[8] = (byte)(ttt);
  enviaSerial[9] = (byte)(s_uD);
  enviaSerial[10] = (byte)(s_uE);
  
  Serial.write(enviaSerial, 11);      // Envia
  //delay(20);
  tt = tt + 1;                // Atualiza contador
}

// Para receber os dados da conexão serial com o computador
void RecebeComputador(){
  int i,t,k;
  String val;
  while (Serial.available()>0){ //checa se tem dados pra ler
    char recieved = Serial.read();
    val +=recieved;
  }
  // Verifica a ordem dos dados no pacote e rearanja caso seja necessário 
  for(i=0;i<=4;i++){
    if(val[i]='#'){ 
      for(k=0;k<=4;k++){
        t=k+i;
        if(t<=4){
          val[k]=val[t];
        }
        else{
          val[k]=val[t-5];
        }
      }
      break;
    }
  }
  
  // Velocidade desejada da roda direita
  refD=(int)val[1];
  RefD=refD;
  if(val[2]=='f'){
    s_refD=0;
  }
  else if(val[2]=='b'){
    s_refD=1;  
    refD=-refD;
  }
  // Velocidade desejada da roda esquerda
  refE=(int)val[3];
  RefE=refE;
  if(val[4]=='f'){
    s_refE=0;
  }
  else if(val[4]=='b'){
    s_refE=1;
    refE=-refE;  
  }
  // Ajusta referência para rad/s
  refD=refD*Wmax/127;
  refE=refE*Wmax/127;
}

void Tempo(){
//  ttt = ttt + 1;
        RecebeComputador();
        EnviaComputador();
}

/* Controlador PID */
void PID_controllers(){
  //Motor Direito -------------------------------------------------------------------------------
  erroD=refD - vD;     // Sinal de erro
  uD=uDa + kpD*(erroD - erroDa) + kdD*(erroD - 2*erroDa + erroDaa)/2  + kiD*T*(erroD + erroDa)/2;
  uDa=uD;              // Atualiza o controle anterior
  erroDaa=erroDa;      // Atualiza o erro anterior anterior
  erroDa=erroD;        // Atualiza o erro anterior
  // Checa o sinal
  if (erroD < 0) {
    s_uD=1;
  }else{
    s_uD=0;
  }
  // Limita em 100
  if (abs(uD) >= 100) {
    uD=100*uD/(abs(uD));
  }
  //Motor Esquerdo ------------------------------------------------------------------------------
  erroE=refE - vE;     // Sinal de erro
  uE=uEa + kpE*(erroE - erroEa) + kdE*(erroE - 2*erroEa + erroEaa)/2  + kiE*T*(erroE + erroEa)/2;
  uEa=uE;              // Atualiza o controle anterior
  erroEaa=erroEa;      // Atualiza o erro anterior anterior
  erroEa=erroE;        // Atualiza o erro anterior
  // Checa o sinal
  if (uE < 0) {
    s_uE=1;
  }else{
    s_uE=0;
  }
  // Limita em 100
  if (abs(uE) >= 100) {
    uE=100*uE/(abs(uE));
  }
}

/* Setup */
void setup(){
  // Inicialização da porta serial
  Serial.begin(57600);
  // Comunicação I2C com o Lego EV3
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(Recebe_Lego);
  Wire.onRequest(Envia_Lego);
  // Interrupção de envio
  MsTimer2::set(500, Tempo);
  MsTimer2::start();
}

void loop(){
    if(flag_lego==3){
        flag_lego=0;
    }
}

// Recebe os sinais do Lego via I2C e salva nas variáveis
void Recebe_Lego(int byteCount){
    int i=0;      // Contador que sincroniza a leitura
    char val; 
    while(Wire.available()>0){
      val=Wire.read();
      recebeLego[i]=val;
      i=i+1;
    }
    // Salva os dados nas variáveis
    vDp=recebeLego[0];
    vD=vDp;
    s_vDp=recebeLego[1];
    if (s_vDp==1){
      vD=-vD;
    }
    vEp=recebeLego[2];
    vE=vEp;
    s_vEp=recebeLego[3];
    if (s_vEp==1){
      vE=-vE;
    }
    // Ajusta velocidade para rad/s
    vD=vD*Wmax/127;
    vE=vE*Wmax/127;
    
    Ud=recebeLego[4];
    Ue=recebeLego[5];
    sUe=recebeLego[6];
    sUd=recebeLego[7];

    PID_controllers();
    ttt++;
    // Filtro -----------------------------------------------------
//    // Carrega dados no buffer
//    bufVd[n] = Vd;
//    bufVe[n] = Ve;
//    // Testa se são as 10 primeiro amostras
//    if (fBufV==0){    
//      Vdf = (bufVd[0] + bufVd[1] + bufVd[2] + bufVd[3] + bufVd[4] + bufVd[5] + bufVd[6] + bufVd[7] + bufVd[8] + bufVd[9])/(n+1);
//      Vef = (bufVe[0] + bufVe[1] + bufVe[2] + bufVe[3] + bufVe[4] + bufVe[5] + bufVe[6] + bufVe[7] + bufVe[8] + bufVe[9])/(n+1);
//    }else{
//      Vdf = (bufVd[0] + bufVd[1] + bufVd[2] + bufVd[3] + bufVd[4] + bufVd[5] + bufVd[6] + bufVd[7] + bufVd[8] + bufVd[9])/10;
//      Vef = (bufVe[0] + bufVe[1] + bufVe[2] + bufVe[3] + bufVe[4] + bufVe[5] + bufVe[6] + bufVe[7] + bufVe[8] + bufVe[9])/10;
//    }
    // ------------------------------------------------------------
    // Atualiza o contador buffer
//    if (n==9){    
//      n = 0; 
//      fBufV = 1;      
//    }else{
//      n = n + 1;
//    }
      flag_lego=flag_lego+1;
}
// Envia sinais para o Lego via I2C
void Envia_Lego(){
    // Cria o vetor a ser enviado ao lego
  enviaLego[0] = (byte)(abs(uD));
  enviaLego[1] = (byte)(s_uD);
  enviaLego[2] = (byte)(abs(uE));
  enviaLego[3] = (byte)(s_uE);    
  Wire.write(enviaLego,8);
}
