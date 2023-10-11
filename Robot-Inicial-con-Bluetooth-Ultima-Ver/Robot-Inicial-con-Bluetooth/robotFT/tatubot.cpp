#include <ArduinoQueue.h>
#include "tatubot.h"
#include "LedControl.h"     
#include "caritas.h"
#include "DFRobotDFPlayerMini.h"
#include "SoftwareSerial.h"


Motor motor0(IN1, IN2, ENA);
Motor motor1(IN4, IN3, ENB);
Led led1(PIN_R_A,PIN_G_A,PIN_B_A);
Led led2(PIN_R_B,PIN_G_B,PIN_B_B);
ArduinoQueue<int> colaOrdenes(CANTIDAD_ORDENES);
LedControl matrizLeds=LedControl(DIN,CLK,CS,MATRICES);
SoftwareSerial mySoftwareSerial(22, 23); // RX, TX
SoftwareSerial myBT(50, 51);
DFRobotDFPlayerMini myDFPlayer; 

int data = 0;
bool avisoMediaCapacidad = false;
bool avisoBajaBateria = false;
bool contador0EnHigh = false;
bool contador1EnHigh = false;
bool banderaDeLineaNegra = true;
bool contemploSiLlegoALineaNegra = true;
int contador0;
int contador1;
float VELOCIDAD = 90;
const int voltajeMax = 25000; // 25V -> 25000mV
int lecturaDigital; // 1023
float voltaje;
int volumenAudio = 27 ; //set value 0-30 

float leerTensionDeBateria() {
  lecturaDigital = analogRead(A7);
  voltaje = map(lecturaDigital, 0, 1023, 0, voltajeMax) / 1000.0;
  return voltaje;
}

float promedioDeTension(){
  int acumuladorDelWhile = 0;
  float suma = 0;
  while(acumuladorDelWhile <= 20)
  {
    suma += leerTensionDeBateria();
    acumuladorDelWhile ++;
  }
  return suma/20;
}

void Robot::iniciar() {
  pinMode(pinEscucharOrdenes, INPUT_PULLUP);
  pinMode(pinAvanzar, INPUT_PULLUP);
  pinMode(pinReversa, INPUT_PULLUP);
  pinMode(pinGiroDerecha, INPUT_PULLUP);
  pinMode(pinGiroIzquierda, INPUT_PULLUP);
  pinMode(pinFinOrdenes, INPUT_PULLUP);
  pinMode(pinBluetooth, INPUT_PULLUP);
  pinMode(SENSOR0, INPUT);
  pinMode(SENSOR1, INPUT);
  Serial.begin(9600);
  myBT.begin(9600);
  mySoftwareSerial.begin(9600 );
  myDFPlayer.begin(mySoftwareSerial);
  iniciarDFPlayerMini();
  motor0.iniciar();
  motor1.iniciar();
  led1.iniciar();
  led2.iniciar();
  matrizLeds.shutdown(0,false);     
  matrizLeds.setIntensity(0,4);    
  matrizLeds.clearDisplay(0);    
  matrizLeds.shutdown(1,false);    
  matrizLeds.setIntensity(1,2);    
  matrizLeds.clearDisplay(1); 
  Serial.print("La bateria esta en: ");
  Serial.println(promedioDeTension());
  delay(50);
}

void Robot::avisarMediaCapacidad(){
  dibujarCaritaSorprendida();
  reproducirSonido(6, 3000);
}

void Robot::avisarBajaBateria(){
  dibujarCaritaDormido();
  reproducirSonido(5, 4000);
}

void Robot::revisarNivelDeBateria(){
  float nivelActual = promedioDeTension();
  if (nivelActual <= 7.75 && nivelActual > 7.23 && avisoMediaCapacidad == false){
      avisarMediaCapacidad();
      avisoMediaCapacidad = true;
    }
  if (nivelActual <= 7.23 && avisoBajaBateria == false){
      avisarBajaBateria();
      avisoBajaBateria = true;
  }
}

void Robot::iniciarDFPlayerMini() {
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
  if (!myDFPlayer.begin(mySoftwareSerial)) {  //Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    //while(true);
  }
  Serial.println(F("DFPlayer Mini online."));
  myDFPlayer.setTimeOut(500); //Set serial communictaion time out 500ms
  //----Set volume----
  myDFPlayer.volume(volumenAudio);  //Set volume value (0~30).
  //----Set different EQ----
  myDFPlayer.EQ(DFPLAYER_EQ_BASS);
  //----Set device we use SD as default----
  myDFPlayer.outputDevice(DFPLAYER_DEVICE_SD);
}

void Robot::reproducirSonido(int sonido, int tiempo) {
  if (mySoftwareSerial.available() > 0){
    myDFPlayer.play(sonido);
    delay(tiempo);
    myDFPlayer.pause();
  }
}

//Movimientos
void Robot::avanzar(int velocidad ) {
  contador0 = 0;
  contador1 = 0;
  contemploSiLlegoALineaNegra = true;
  banderaDeLineaNegra = true;
  movimientoAdelante(velocidad);
}

void Robot::retroceder(int velocidad) {
  contador0 = 0;
  contador1 = 0;
  contemploSiLlegoALineaNegra = true;
  banderaDeLineaNegra = true;
  movimientoReversa(velocidad);
}

void Robot::girarDerecha() {
  contador0 = 0;
  contador1 = 0;
  movimientoDerecha();
}

void Robot::girarIzquierda() {
  contador0 = 0;
  contador1 = 0;
  movimientoIzquierda();
}

void Robot::movimientoAdelante(int velocidad)
{
  while(contemploSiLlegoALineaNegra)
  {
    motor0.adelante(velocidad);
    motor1.adelante(velocidad);
    //led1.rgb('g');
    //led2.rgb('g');
    detectarLineaNegraEIgualar(velocidad);
    }
    while(contador0 <= 13  && contador1<= 13)
    {
      motor0.adelante(velocidad);
      motor1.adelante(velocidad);
      //led1.rgb('g');
      //led2.rgb('g');
      contador();
    }
    pararRobot();
}

void Robot::movimientoReversa(int velocidad)
{
  while(contemploSiLlegoALineaNegra)
  {
    motor0.atras(velocidad);
    motor1.atras(velocidad);
    //led1.rgb('r');
    //led2.rgb('r');
    detectarLineaNegraEIgualarEnReversa(velocidad);
    }
  while(contador0 <= 6 && contador1 <= 6) {
    motor0.atras(velocidad);
    motor1.atras(velocidad);
    //led1.rgb('r');
    //led2.rgb('r');
    contador();
  }
    pararRobot();
}

void Robot::movimientoDerecha()
{
  //led 1 izq mirando de atras
  while (contador1 < 15) {
      motor0.atras(200);
      motor1.adelante(200);
      delay(15);
      //led1.rgb('b');
      contador();
  }
  pararRobot();
  //led1.rgb('a');
}

void Robot::movimientoIzquierda()
{
  //led 2 derecha mirando de atras
  while (contador0 < 15) {
      motor0.adelante(200);
      motor1.atras(200);
      delay(15);
      //led2.rgb('b');
      contador();
    }
  pararRobot();
  //led2.rgb('a');
}

void Robot::pararRobot() {
  motor0.parar();
  motor1.parar();
  //led1.rgb('a');
  //led2.rgb('a');
}

//correcciones del robot y los sensores, "A tener en cuenta" todos los movimientos e indicadores de derecha e izquierda se hicieron mirando el robot desde atrás o sea la espalda del robot
void Robot::autocorregir() {
 if (contador0 > contador1) {
  igualarRuedaAdelantada0();
 }
 else if (contador1 > contador0) {
  igualarRuedaAdelantada1();
 }
}

void Robot::igualarRuedaAdelantada0() 
{
  while (contador1 != contador0) {
    motor0.parar();
    motor1.adelante(75);
    contador();
  }
}

void Robot::igualarRuedaAdelantada1() 
{
  while (contador0 != contador1) {
    motor1.parar();
    motor0.adelante(75);
    contador();
  }
}

void Robot::autocorregirReversa() {
 if (contador0 > contador1) {
  igualarRuedaAdelantadaReversa0();
 }
 else if (contador1 > contador0) {
  igualarRuedaAdelantadaReversa1();
 }
}

void Robot::igualarRuedaAdelantadaReversa0() 
{
  while (contador0 != contador1) {
    motor0.parar();
    motor1.atras(85);
    contador();
  }
}

void Robot::igualarRuedaAdelantadaReversa1() 
{
  while (contador1 != contador0) {
    motor1.parar();
    motor0.atras(85);
    contador();
  }
}

//En la función contador y todas las funciones que utilicen un contador son funciones que trabajan con los sensores de tacómetro colocados en las ruedas.
void Robot::contador() {
  if (digitalRead(SENSOR1) == LOW && contador1EnHigh == false)  
  {
      contador1++; 
      contador1EnHigh = true;
  }
  if (digitalRead(SENSOR1) == HIGH)
  {
      contador1EnHigh = false;
  }
  if (digitalRead(SENSOR0) == LOW && contador0EnHigh == false )
  {
      contador0++;  
      contador0EnHigh = true;
  }
  if (digitalRead(SENSOR0) == HIGH)
  {
       contador0EnHigh = false;
  }
}

//Todas las funciones que utilicen detectar líneas negras o color blanco son funciones que trabajan con los sensores ópticos tcrt5000 que están colocados debajo del robot
//es importante que los sensores estén lo más adelante posible y lo más parejo posible ya que influye mucho en la corrección y el robot podria funcionar de forma errática
void Robot::detectarLineaNegraEIgualarEnReversa(int velocidad) {
  if(banderaDeLineaNegra == true){
    if (digitalRead(SENSORIZQUIERDO) == HIGH && digitalRead( SENSORDERECHO) == LOW )
    {
      moverMotor0HastaLineaNegra(velocidad);
      banderaDeLineaNegra = false;
      contemploSiLlegoALineaNegra = false;
    }
    if (digitalRead(SENSORDERECHO) == HIGH && digitalRead(SENSORIZQUIERDO) == LOW && banderaDeLineaNegra == true )
    {
      moverMotor1HastaLineaNegra(velocidad);
      banderaDeLineaNegra = false;
      contemploSiLlegoALineaNegra = false;
    }
  }
}
///// linea negra en reversa
void Robot::moverMotor0HastaLineaNegra(int velocidad) {
  while (digitalRead(SENSORDERECHO)== LOW) {
      motor0.atras(velocidad);
      motor1.parar();
      delay(15);
  }
  //rueda parada atras en blanco se adelanta al negro
    while(digitalRead(SENSORIZQUIERDO) == LOW){
        motor0.parar();
        motor1.adelante(velocidad);
     }
  enderezarMotoresDesdeLineaNegraEnReversa(velocidad);
  pararRobot();
  delay(500);
  enderezarSiSePasanLosMotoresABlancoEnReversa(velocidad);
}

void Robot::moverMotor1HastaLineaNegra(int velocidad) {
  while (digitalRead(SENSORIZQUIERDO) == LOW){
      motor0.parar();
      motor1.atras(velocidad);
      delay(15);
  }
  //rueda parada atras en blanco se adelanta al negro
  while(digitalRead(SENSORDERECHO)== LOW){
      motor0.adelante(velocidad);
      motor1.parar();
    }
  enderezarMotoresDesdeLineaNegraEnReversa(velocidad);
  pararRobot();
  delay(500);
  enderezarSiSePasanLosMotoresABlancoEnReversa(velocidad);
}

void Robot::detectarLineaNegraEIgualar(int velocidad) {
  if(banderaDeLineaNegra == true){
    if (digitalRead(SENSORIZQUIERDO) == HIGH && digitalRead(SENSORDERECHO) == LOW )
    {
      adelantarMotorAtrasado0(velocidad);
      banderaDeLineaNegra = false;
      contemploSiLlegoALineaNegra = false;
    }
    if (digitalRead(SENSORDERECHO) == HIGH && digitalRead(SENSORIZQUIERDO) == LOW && banderaDeLineaNegra == true )
    {
      adelantarMotorAtrasado1(velocidad);
      banderaDeLineaNegra = false;
      contemploSiLlegoALineaNegra = false;
    }
  }
}

void Robot::adelantarMotorAtrasado0(int velocidad){
    //Caso que la rueda izquierda llega primero a la línea negra 
    while (digitalRead(SENSORDERECHO)== LOW) {
      motor0.adelante(velocidad);
      motor1.parar();
      delay(10);
    }
    //Si la rueda parada se pasó hacia adelante cuando estaba tratando de enderezar el robot y quedó en blanco, entonces ésta retrocede al negro
    while(digitalRead(SENSORIZQUIERDO)== LOW){
      motor1.atras(velocidad);
      motor0.parar();
      }
    enderezarMotoresDesdeLineaNegra(velocidad);
  pararRobot();
  delay(500);
  enderezarSiSePasanLosMotoresABlanco(velocidad);
}

void Robot::adelantarMotorAtrasado1(int velocidad) {
  //Caso que la rueda derecha llega primero a la  línea negra
  while (digitalRead(SENSORIZQUIERDO) == LOW){
    motor1.adelante(velocidad);
    motor0.parar();
    delay(10);
  }
  //Si la rueda parada se pasó hacia adelante cuando estaba tratando de enderezar el robot y quedó en blanco, entonces ésta retrocede al negro
  while(digitalRead(SENSORDERECHO) == LOW){
      motor0.atras(velocidad);
      motor1.parar();
  }
  enderezarMotoresDesdeLineaNegra(velocidad);
  pararRobot();
  delay(500);
  enderezarSiSePasanLosMotoresABlanco(velocidad);
}

void Robot::enderezarMotoresDesdeLineaNegra(int velocidad) {
  while(digitalRead(SENSORIZQUIERDO)== HIGH){
      motor0.parar();
      motor1.adelante(velocidad);
  }
  while(digitalRead(SENSORDERECHO)== HIGH){
      motor0.adelante(velocidad);
      motor1.parar();
 }
 pararRobot();
 delay(50);
}

void Robot::enderezarMotoresDesdeLineaNegraEnReversa(int velocidad) {
  while(digitalRead(SENSORIZQUIERDO)== HIGH){
      motor0.parar();
      motor1.atras(velocidad);
  }
  while(digitalRead(SENSORDERECHO)== HIGH){
      motor0.atras(velocidad);
      motor1.parar();
 }
 pararRobot();
 delay(50);
}
  
void Robot::enderezarSiSePasanLosMotoresABlanco(int velocidad) {
  //Pregunta si los 2 motores se pasaron y frenaron en blanco pasada la línea negra
  if (digitalRead(SENSORDERECHO)== LOW && digitalRead(SENSORIZQUIERDO)== LOW) {
    while (digitalRead(SENSORDERECHO)== LOW) {
      motor0.atras(velocidad);
      motor1.parar();
    }
    while (digitalRead(SENSORIZQUIERDO)== LOW) {
      motor0.parar();
      motor1.atras(velocidad);
    }
   pararRobot();
   delay(10);
   enderezarMotoresDesdeLineaNegra(velocidad);
  }
}

void Robot::enderezarSiSePasanLosMotoresABlancoEnReversa(int velocidad) {
  //Pregunta si los 2 motores se pasaron y frenaron en blanco pasada de línea negra
  if (digitalRead(SENSORDERECHO)== LOW && digitalRead(SENSORIZQUIERDO)== LOW) {
    while (digitalRead(SENSORDERECHO)== LOW) {
      motor0.adelante(velocidad);
      motor1.parar();
    }
    while (digitalRead(SENSORIZQUIERDO)== LOW) {
      motor0.parar();
      motor1.adelante(velocidad);
    }
   pararRobot();
   delay(10);
   enderezarMotoresDesdeLineaNegraEnReversa(velocidad);
  }
}

//Calcula la tensión de las baterias y se arma un promedio para avisar por audios
void Robot::calcularCasosDeVelocidad() {
  /*
   Vmáx= 8.5
   Vmin= 7.1
   Vmáx - Vmin = DiferenciaDeVoltaje / 4
   0% = - de 7V
   25% = 7.10V
   50% = 7.31V
   75% = 8.18V
   100% = 8.5
  */
  float valorDeTension = promedioDeTension();
  //avisar al 10% que tiene sueño 7.23v audio 8
  //avisar que esta al 50% 7.75v audio 9
  //maximo de tension = 8.4 y minimo = 7.1
  if(7.10 <= valorDeTension  && valorDeTension  < 7.31 ){
    VELOCIDAD = 130;
  }
  else if(7.31 <= valorDeTension && valorDeTension  <= 8.18 ){
    VELOCIDAD = 110;
  }
  else if (8.18 <= valorDeTension && valorDeTension  <= 8.5 ){
    VELOCIDAD = 90;
  }
  else {
    VELOCIDAD = 0;
  }
}

//Guardado de órdenes en la cola a partir de presionar el boton de inicio
void Robot::escucharOrdenes() {
bool reproduciSonido = false;

while (colaOrdenes.itemCount() <= CANTIDAD_ORDENES && digitalRead(pinFinOrdenes) != LOW) {
  if (digitalRead(pinAvanzar) == LOW) {
        dibujarCaritaSorprendida();          
        colaOrdenes.enqueue(1);
        Serial.print("Orden para avanzar en cola \n");
        reproducirSonido(11, 150); 
        dibujarCaritaFeliz();      
  }
  delay(30);
  if (digitalRead(pinReversa) == LOW) {
        dibujarCaritaSorprendida();           
        colaOrdenes.enqueue(2);
        Serial.print("Orden para retroceder en cola \n");
        reproducirSonido(11, 150); 
        dibujarCaritaFeliz(); 
  }
  delay(30);
  if (digitalRead(pinGiroDerecha) == LOW) {  
        dibujarCaritaSorprendida();         
        colaOrdenes.enqueue(3);
        Serial.print("Orden para girar a la derecha en cola \n");
        reproducirSonido(11, 150); 
        dibujarCaritaFeliz(); 
  }
  delay(30);
  if (digitalRead(pinGiroIzquierda) == LOW) {    
        dibujarCaritaSorprendida();       
        colaOrdenes.enqueue(4);
        Serial.print("Orden para girar a la izquierda en cola \n");
        reproducirSonido(11, 150); 
        dibujarCaritaFeliz(); 
  }
  delay(30);
  if (digitalRead (PIN_INICIO_ORDENES) == LOW) {
    while(colaOrdenes.itemCount() > 0)
    {
      colaOrdenes.dequeue();
    }
    Serial.println("resetee todas las ordenes");
  }
  delay(30);
  if (colaOrdenes.itemCount() == CANTIDAD_ORDENES && reproduciSonido == false) { 
    dibujarCaritaNormal();
    reproducirSonido(10, 3000);
    reproduciSonido = true;
  }
} 
reproducirSonido(3, 750);
Serial.print("Se ejecutan las ordenes \n");
Serial.print(colaOrdenes.itemCount());
ejecutarTodasLasOrdenes();
}

//Conexión de Bluetooth y recepción de órdenes
void Robot::BtStandBy(int velocidad) {
estoyEnBtStandBy = true;
bool banderaBt = true;

if(banderaBt == true){
  while (!myBT.available() && estoyEnBtStandBy = false) {
    //colocar sonido de bluetooth no conectado o fuera de alcance
    //reproducirSonido(11, 150);
    Serial.println("Conexion a Bluetooth no disponible");
    led1.rgb('r');
    delay(500);
    led1.rgb('a');
  }
  estoyEnBtStandBy = true;
  Serial.println(myBT.available());
  if(myBT.available() > 0) {
  data = myBT.read();
  //colocar sonido de bluetooth conectado
  reproducirSonido(11, 150);
  Serial.println("Estoy conectado a Bluetooth");
  led1.rgb('g');
      if (data == 'F'){
      //avanzar
        dibujarCaritaFeliz();
        reproducirSonido(11, 150);
      }
      if (data == 'B'){
        //retroceder
        dibujarCaritaEntusiasmada();
        reproducirSonido(11, 150);
      }
      if (data == 'R'){
        //girar A La Derecha
        dibujarCaritaEnojada();
        reproducirSonido(11, 150);
      }
      if (data == 'L'){
        //girar A La Izquierda
        avisarBajaBateria();
      }
      if (data=='V'){
        dibujarCaritaEntusiasmada();
        reproducirSonido(2, 1900); 
        rutinaBtProgramable(200);
      }
      if (data == 'S'){
        pararRobot();
      }
  }
  if (digitalRead(PIN_BLUETOOTH) == HIGH) { 
    /*La bandera y esta función se ejecutan desde acá adentro ya que estando dentro de la función bluetooth esta es la unica 
    forma de que salga de la conexión de lo contario nunca saldría de la conexión*/         
      banderaBt = false;
      estoyEnBtStandBy = false;
      //Colocar sonido de bluetooth apagado
      reproducirSonido(11, 150);
      led1.rgb('a');
      Serial.print("Corte conexion con Bluetooth");
      reproducirSonido(17, 1900);
      dibujarCaritaFeliz();
  }
  }
}

//Esta rutina guarda órdenes en una cola hasta que se ejecuten
void Robot::rutinaBtProgramable(int velocidad){
  bool banderaBt = true;
  bool saliYaEjecute = false;
  
   while(banderaBt == true && saliYaEjecute== false){
     Serial.println(myBT.available());
     if(myBT.available() > 0){
      data = myBT.read();
      Serial.println("Estoy conectado a BT");
     
       if (data == 'F'){
        //avanzar
          colaOrdenes.enqueue(1);
          dibujarCaritaFeliz();
          reproducirSonido(11, 150);
        }
        if (data == 'B'){
          //retroceder
          colaOrdenes.enqueue(2);
          dibujarCaritaFeliz();
          reproducirSonido(11, 150);
        }
        if (data == 'R'){
          //girar A La Derecha
          colaOrdenes.enqueue(3);
          dibujarCaritaFeliz();
          reproducirSonido(11, 150);
        }
        if (data == 'L'){
          //girar A La Izquierda
          colaOrdenes.enqueue(4);
          dibujarCaritaFeliz();
          reproducirSonido(11, 150);
        }
        if (data=='V'){
         ejecutarTodasLasOrdenes();
         saliYaEjecute = true;
        }
        if (data == 'S'){
          pararRobot();
        }
      }
   }
}

//Ejecución de órdenes guardadas en la cola
void Robot::ejecutarTodasLasOrdenes() {
  int cantidadOrdenes = colaOrdenes.itemCount(); // Pasa el numero de ordenes acumuladas en la cola
  delay(1000);
  for(int i=0; i<cantidadOrdenes; i++) // Marca cuantas veces debe hacer la ejecucion de la primer orden actual de la cola 
  {
        Serial.print(colaOrdenes.itemCount());
        dibujarCaritaEntusiasmada();
        ejecutarOrden(colaOrdenes.dequeue()); // retorna el primero de la cola y lo remueve, esto se hace tanta veces como ordenes se les hayan programado
        dibujarCaritaFeliz();
        delay(1000);
  }
  dibujarCaritaFeliz();
  reproducirSonido(7, 2000); // Sonido cuando termina de ejecutar ordenes
}

void Robot::ejecutarOrden(byte caso) {
  switch(caso) {
    case 1:
      avanzar(VELOCIDAD);
      break;
    case 2:
      retroceder(VELOCIDAD);
      break;
    case 3:
      girarDerecha();
      break;
    case 4:
      girarIzquierda();
      break;
  }
}

//Dibuja las caritas del robot
void Robot::dibujarCarita(byte ojos[8], byte boca[8]) {     
  for (int i = 0; i < 8; i++)     
  {
    matrizLeds.setRow(0,i,ojos[i]);   
  }
  for (int i = 0; i < 8; i++)     
  {
    matrizLeds.setRow(1,i,boca[i]);    
  }
}

void Robot::dibujarCaritaFeliz() { dibujarCarita(ojoFeliz,  ojoFeliz); }
void Robot::dibujarCaritaSorprendida() { dibujarCarita(ojoDefault,  ojoDefault); }
void Robot::dibujarCaritaEntusiasmada() { dibujarCarita(ojoCorazonCabeza,  ojoCorazonCabeza); }
void Robot::dibujarCaritaTriste() { dibujarCarita(ojoCerrado,  ojoCerrado); }
void Robot::dibujarCaritaEnojada() { dibujarCarita(ojoEnojadoDerCabeza,  ojoEnojadoIzqCabeza); }
void Robot::dibujarCaritaGuiniando() { dibujarCarita(ojoFeliz,  ojoCerrado); }
void Robot::dibujarCaritaDormido() {dibujarCarita(dormidoOjo, dormidoOjo); }
void Robot::dibujarCaritaNormal() {dibujarCarita(normalOjo, normalOjo);}
void Robot::despertar() {
  myDFPlayer.play(1);
  dibujarCaritaTriste();
  delay(300);
  dibujarCaritaSorprendida();
  delay(500);
  dibujarCaritaNormal();
  delay(300);
  dibujarCaritaFeliz();
  delay(1000);
  dibujarCaritaGuiniando();
  delay(250);
  dibujarCaritaFeliz();
  delay(1000);
  myDFPlayer.pause();
}

//---------------------------------------------
Motor::Motor(byte pinAtras, byte pinAdelante, byte pinVelocidad) {
  this->pinAtras = pinAtras;
  this->pinAdelante = pinAdelante;
  this->pinVelocidad = pinVelocidad;
}

void Motor::iniciar() {
  pinMode(pinAtras, OUTPUT);
  pinMode(pinAdelante, OUTPUT);
  pinMode(pinVelocidad, OUTPUT);
  this->parar();
}

void Motor::adelante(int velocidad) {
  analogWrite(pinVelocidad, velocidad);
  digitalWrite(pinAtras, LOW);
  digitalWrite(pinAdelante, HIGH);
}

void Motor::atras(int velocidad) {
  analogWrite(pinVelocidad, velocidad);
  digitalWrite(pinAdelante, LOW);
  digitalWrite(pinAtras, HIGH);
}

void Motor::parar() {
  analogWrite(pinVelocidad, 0);
}


//-------------------------------------------------------
Led::Led(byte ledR, byte ledG, byte ledB) {
  this->ledR = ledR;
  this->ledG = ledG;
  this->ledB = ledB;
}

void Led::iniciar() {
  pinMode(ledR, OUTPUT);
  pinMode(ledG, OUTPUT);
  pinMode(ledB, OUTPUT);
  this->rgb('a');
}


void Led::rgb(char color) {
  switch (color) {
    case 'r': //red
      digitalWrite(ledR, LOW);
      digitalWrite(ledG, HIGH);
      digitalWrite(ledB, HIGH);
      break;
    case 'g': //green
      digitalWrite(ledR, HIGH);
      digitalWrite(ledG, LOW);
      digitalWrite(ledB, HIGH);
      break;
    case 'y': //yellow
      digitalWrite(ledR, LOW);
      digitalWrite(ledG, LOW);
      digitalWrite(ledB, HIGH);
      break;
    case 'b': //blue
      digitalWrite(ledR, HIGH);
      digitalWrite(ledG, HIGH);
      digitalWrite(ledB, LOW);
      break;
    case 'a': //apagado
      digitalWrite(ledR, HIGH);
      digitalWrite(ledG, HIGH);
      digitalWrite(ledB, HIGH);
      break;
  }
}
