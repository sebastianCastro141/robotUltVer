/*
  Programa para manejar el robot Tatu-bot de Futhur Tech
 */
#include "tatubot.h"
#include "SoftwareSerial.h"
Robot tatubot;



void setup(){
  Serial.begin(9600);
  tatubot.iniciar();
  tatubot.despertar();
}

void loop(){
  //tatubot.estoyEnBtStandBy = false;
  if (digitalRead(PIN_INICIO_ORDENES) == LOW) {
    Serial.print("Escuchando ordenes \n");
    tatubot.dibujarCaritaEntusiasmada();
    tatubot.reproducirSonido(2, 1900);         
    tatubot.escucharOrdenes();
  }
  if (digitalRead(PIN_ORDEN_AVANZAR) == LOW) {          
    tatubot.dibujarCaritaFeliz();
    tatubot.reproducirSonido(11, 150);
  }
  if (digitalRead(PIN_ORDEN_REVERSA) == LOW) {          
    tatubot.dibujarCaritaSorprendida();
    tatubot.reproducirSonido(11, 150);
  }
  if (digitalRead(PIN_ORDEN_GIRO_DERECHA) == LOW) { 
    tatubot.dibujarCaritaGuiniando();         
    tatubot.reproducirSonido(11, 150); 
  }
  if (digitalRead(PIN_ORDEN_GIRO_IZQUIERDA) == LOW) {
    tatubot.dibujarCaritaNormal();          
    tatubot.reproducirSonido(11, 150);
  }
  if (digitalRead(PIN_FIN_ORDENES) == LOW) {          
    tatubot.dibujarCaritaEntusiasmada();
    tatubot.reproducirSonido(3, 750); 
  }
  if (digitalRead(PIN_BLUETOOTH) == LOW && tatubot.estoyEnBtStandBy == false) {
    //colocar sonido de bluetooth encendido 
    tatubot.reproducirSonido(11, 150); 
    tatubot.BtStandBy(200);
  }
  tatubot.calcularCasosDeVelocidad();
  tatubot.revisarNivelDeBateria();
}
