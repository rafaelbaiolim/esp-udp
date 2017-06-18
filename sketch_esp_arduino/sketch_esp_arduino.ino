#include "WiFiEsp.h"
#include "WiFiEspUdp.h"
#include "SoftwareSerial.h"
//#include "Ultrasonic.h"

//********************** Declaração de Variáveis Globais **********************
/** ARDUINO **/
#define CH_PD 5 //sinal de controle do CH_PD
#define RST 6 //pino de controle do RESSET
SoftwareSerial Serial1(3, 4); // (RX, TX) Faz a simulação do Serial1

/*** ULTRASSONICO **/
#define PIN_TRIGGER 11
#define PIN_ECHO 10

/** MOTORES **/
#define MOTOR_1 A1
#define MOTOR_2 A2
#define MOTOR_3 A3

/** CONTRALADOR DOS MOTORES **/
#define ON_MOTOR_1 "ON_MOTOR_1"
#define ON_MOTOR_2 "ON_MOTOR_2"
#define ON_MOTOR_3 "ON_MOTOR_3"
#define OFF_MOTOR_1 "OFF_MOTOR_1"
//defina outras constantes de acionamento aqui

/*** GLOBAIS **/
char ssid[] = "GVT";                   // SSID da rede
char pass[] = "17151715";             // senha do SSID
char ipSocket[] = "192.168.25.124";  // IP utilizado no SOCKET
unsigned int serverPort = 9876;     // porta do servidor
unsigned int localPort  = 9875;    // porta de recebimento (client)
char packetBuffer[255];           // Buffer para processar os pacotes recebidos
int status = WL_IDLE_STATUS;     // Constante definida pela biblioteca, status AGUARDANDO_CONEXAO
WiFiEspUDP Udp;                 //Instancia do tipo UDP implementado pela biblioteca
//Ultrasonic ultrasonic(PIN_TRIGGER, PIN_ECHO); //biblioteca para utilizar o utrassonico

//********************** Algoritmo **********************

void setup() {
  delay(100);
  inicializarESP();
  //inicializarMotores(); //Método que irá setar os pinMode dos Motores
  Serial.begin(9600);   // Incia a comunicação serial do Arduino
  Serial.println(F(">Inicializando...")); //println(F(STRING)) é utilizado para otimização de memória
  conectarNoWifi();   // Inicia a conexção WI-FI
  Udp.begin(localPort);   // Inicia a conexão com o socket via UDP
}

void loop() {
  //Envia as informações do módulo ultrassonico
  enviarDadosUltrassonico();
  
  //Pega um pacote do udp se estiver na fila
  int packetSize = Udp.parsePacket();

  // Se existir um pacote vindo do servidor, processa
  if (packetSize) {
    processarPacoteUDP();
  } 
}

//********************** Métodos de Inicialização **********************
void inicializarESP() {
  pinMode(CH_PD, OUTPUT);
  pinMode(RST, OUTPUT);
  digitalWrite(CH_PD, HIGH);
  //reseta a esp para garantir que o buffer não esteja com "lixo"
  delay(100);
  digitalWrite(RST, LOW);
  delay(100);
  digitalWrite(RST, HIGH); //RST em alto, funcionamento normal
  Serial1.begin(9600); //Inicia a comunicação serial do ESP
  WiFi.init(&Serial1); // Inicializa a biblioteca
}

//void inicializarMototes(){}

void conectarNoWifi() {
  // Se não conseguir a comunicação com a esp, trava a execução
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println(F("Esp não inciada corretamente, tente resetar."));
    while (true);
  }

  // Enquanto o status for diferente de "CONECTADO"
  while ( status != WL_CONNECTED) {
    Serial.println(F("Tentando se conectar a rede: "));
    Serial.println(ssid);
    // Verifica se a conexão foi bem sucedida
    status = WiFi.begin(ssid, pass);
  }
  Serial.println(F("Rede conectada."));
}

//********************** Métodos de Envio **********************

//** Método principal de envio dos pacotes
void enviarDadosViaUDP(const char *dados) {

  
  Serial.println(F("\n>>Enviado"));
  Serial.print(dados);
  
  Udp.beginPacket(ipSocket, serverPort); //abre o socket utilizando o ip:porta
  Udp.write(dados); //empacota e envia os bits
  Udp.endPacket(); //finaliza o envio

  Serial.print("\n");
  delay(1000); // ******>>>> Aguarda um 1 segundo antes do próximo envio, remover na versão final <<<******
}

void enviarDadosUltrassonico() {
  float cmMsec;
  //long microsec = ultrasonic.timing();
  //cmMsec = ultrasonic.convert(microsec, Ultrasonic::CM);
  cmMsec = 6.550; //valor simulado de distancia em cm
  char result[3]; // Buffer para enviar o float como um char*
  dtostrf(cmMsec, 3, 3, result);
  enviarDadosViaUDP(result);
}

//********************** Métodos de Recebimento **********************

//Processa os dados recebidos no pacote de até 255 bytes(size do buffer)
void processarPacoteUDP() {
  int len = Udp.read(packetBuffer, 255);
  if (len > 0) {
    packetBuffer[len] = 0;  //reseta o buffer do UDP
  }
  String comando = (String) packetBuffer;

  Serial.println(F("\n>>Recebido:"));
  Serial.print(packetBuffer);

  //recebe as solicitação de comandos do servidor
  //e envia uma resposta a partir do arduino
  if (comando.equals(ON_MOTOR_1)) {
    //digitalWrite(MOTOR_1, HIGH);
    enviarDadosViaUDP("MOTOR 1 LIGADO");
  }

  if (comando.equals(ON_MOTOR_2)) {
    //digitalWrite(MOTOR_2, HIGH);
    enviarDadosViaUDP("MOTOR 2 LIGADO");
  }

  if (comando.equals(ON_MOTOR_3)) {
    //digitalWrite(MOTOR_3, HIGH);
    enviarDadosViaUDP("MOTOR 3 LIGADO");
  }

  if (comando.equals(OFF_MOTOR_1)) {
    //digitalWrite(MOTOR_1, HIGH);
    enviarDadosViaUDP("MOTOR 1 DESLIGADO");
  }

}




