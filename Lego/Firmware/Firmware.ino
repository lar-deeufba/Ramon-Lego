#include <Wire.h>
#include<MsTimer2.h>
#define SLAVE_ADDRESS 0x04

// Variáveis de comunicação
int val;
uint8_t c[]="DEXTERIN";
char data[10];
byte dados[8] = {0, 0, 0, 0, 0, 0, 0, 0};
byte buf[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
byte U_lego[8] = {0, 0, 0, 0, 0, 0, 0, 0};
// Velocidades Lidas
double Vd, Ve;
// Sinais das velocidades
double sVd, sVe;
// Velocidades desejadas
double Ref_Vd, Ref_Ve;
double vDref, vEref;
double VDref, VEref;
double sVDref, sVEref;
// Sinais de controle aplicados
double Ud, Ue, sUe, sUd;
// Contador para indicar que houve 5 iteraçãoes com o lego
// o que equivale a 500ms (a temporização é feita no lego)
int flag_lego;
long double tt = 1;                            // Contador para debug
long double ttt = 0;                           // Contador para debug
double Wmax = 17.3398;
/* ------------------------- Funções ------------------------- */
// Envio de dados pela conexao serial
void EnviaComputador()
{
  // Carrega o buffer de envio
  buf[0] = (byte)'@';
  buf[1] = (byte)(sVd);
  buf[2] = (byte)(abs(Vd));
  buf[3] = (byte)(abs(Ud));
  buf[4] = (byte)(sVe);
  buf[5] = (byte)(abs(Ve));
  buf[6] = (byte)(abs(Ue));
  buf[7] = (byte)(tt);
  buf[8] = (byte)(ttt);
  buf[9] = (byte)(sUd);
  buf[10] = (byte)(sUe);
  
  // Envia
  Serial.write(buf, 11);
  tt = tt + 1;                             // Atualiza contador
}

// Para receber os dados da conexão Serial
void RecebeComputador(){
  int i,t,k;
  String val;
  while (Serial.available()>0) { //checa se tem dados pra ler
    char recieved = Serial.read();
    val +=recieved;
  }
  // Verifica a ordem dos dados no pacote
  // e rearanja caso seja necessário 
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
  Ref_Vd = (int)val[1];
  if(val[2]=='f'){
    sVDref = 0;
  }
  else if(val[2]=='b'){
    sVDref = 1;  
  }
  // Velocidade desejada da roda esquerda
  Ref_Ve = (int)val[3];
  if(val[4]=='f'){
    sVEref = 0;
  }
  else if(val[4]=='b'){
    sVEref = 1;  
  }
  
  VDref = (Ref_Vd);
  VEref = (Ref_Ve);
  // Cria o vetor a ser enviado ao lego
  U_lego[0] = (byte)(VEref);
  U_lego[1] = (byte)(sVEref);
  U_lego[2] = (byte)(VDref);
  U_lego[3] = (byte)(sVDref);    
}

void Tempo(){
  ttt = ttt + 1;
}

/* Setup */
void setup()
{
// Comentado para usar com o XBee  
//  int i;
  // Configuração
//  for (i = 5; i <= 8; i++) {
//    pinMode(i, OUTPUT);
//  }
  // Inicialização da porta serial
  Serial.begin(57600);
//  data[0]='~';
//  data[2]='H';
//  data[4]='H';
//  data[5]='#';
  // Comunicação I2C com o Lego EV3
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(Recebe_Lego);
  Wire.onRequest(Envia_Lego);
  // Inicialização das velocidades desejadas
  vEref = 0;
  vDref = 0;
  // Interrupção de envio
  MsTimer2::set(500, Tempo);
  MsTimer2::start();
}

void loop()
{
  if(flag_lego == 3)
   {
     RecebeComputador();
     EnviaComputador();
     flag_lego = 0;
   }
}

// Recebe os sinais do Lego via I2C e salva nas variáveis
void Recebe_Lego(int byteCount)
{
    // Contador que sincroniza a leitura
    int i = 0;
    while(Wire.available()>0)
    {
      val = Wire.read();
      dados[i] = val;
      i = i + 1;
    }
    // Salva os dados nas variáveis
    Ve = dados[0];
    sVe = dados[1];
    if (sVe==1){
      sVe = -1;
    }
    Vd = dados[2];
    sVd = dados[3];
    if (sVd==1){
      sVd = -1;
    }
    Ue = dados[4];
    Ud = dados[5];
    sUe = dados[6];
    if (sUe==1){
      sUe = -1;
    }
    sUd = dados[7];
    if (sUd==1){
      sUd = -1;
    }
    // Atualiza o contador
    flag_lego = flag_lego + 1;
}
// Envia sinais para o Lego via I2C
void Envia_Lego()
{
  Wire.write(U_lego,8);
}
