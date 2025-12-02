#include "diff_car.h"
#include "mqtt.h"
#define COMUM_SPEED 0.4f

void MqttTask::girE(int tempoGiro){
  diffCar.left_velocity_target = -COMUM_SPEED;
  diffCar.right_velocity_target = COMUM_SPEED;
}

void MqttTask::girD(int tempoGiro){
  diffCar.left_velocity_target = COMUM_SPEED;
  diffCar.right_velocity_target = -COMUM_SPEED;
}

void MqttTask::para(){
  diffCar.mode = 0;
  diffCar.left_velocity_target = 0.0f;
  diffCar.right_velocity_target = 0.0f;
}

void MqttTask::movF(int tempoMov){
  diffCar.mode = 0;
  diffCar.left_velocity_target = COMUM_SPEED;
  diffCar.right_velocity_target = COMUM_SPEED;
}

void MqttTask::movT(int tempoMov){
  diffCar.mode = 0;
  diffCar.left_velocity_target = -COMUM_SPEED;
  diffCar.right_velocity_target = -COMUM_SPEED;
}

void MqttTask::rotD_F(int tempoMov) {
  diffCar.left_velocity_target = 0;
  diffCar.right_velocity_target = COMUM_SPEED;
}

void MqttTask::rotE_F(int tempoMov) {
  diffCar.left_velocity_target = COMUM_SPEED;
  diffCar.right_velocity_target = 0;
}

void MqttTask::rotD_T(int tempoMov) {
  diffCar.left_velocity_target = 0;
  diffCar.right_velocity_target = -COMUM_SPEED;
}

void MqttTask::rotE_T(int tempoMov) {
  diffCar.left_velocity_target = -COMUM_SPEED;
  diffCar.right_velocity_target = 0;
}

void MqttTask::ligaLeds(){

}

void MqttTask::acendeLeds(int led1, int led2, int led3, int led4, int led5, int led6, int led7, int led8){

}   

void MqttTask::desligaLeds(){

}

void MqttTask::coreo(){
  unsigned long timeLimit = 0;
  unsigned long tempo = millis();
  Serial.print("TEMPO1: ");
  Serial.println(tempo);
  delay(500);
  timeLimit = 190000 + millis() + 12000;
  while( tempo <= timeLimit ){

    //Movimento 1 Abaixo
    ligaLeds();
    movF(3000);
    
    acendeLeds(0,0,1,1,1,1,1,0);
    movT(6000);
    
    acendeLeds(1,1,1,0,0,0,1,1);
    movF(3000);
    
    desligaLeds();
    girD(500);
    
    ligaLeds();
    movF(3000);

    desligaLeds();
    girE(500);
    
    acendeLeds(1,1,1,0,0,0,0,1);
    rotE_F(500);
    movF(500);

    acendeLeds(1,1,1,0,0,0,1,1);
    rotD_F(500);
    movF(500);

    acendeLeds(1,1,1,0,0,0,0,1);
    rotE_F(500);
    movF(500);

    acendeLeds(1,1,1,0,0,0,1,1);
    rotD_F(500);
    movF(500);

      ////Change Direction of Waves to Down////

    acendeLeds(0,0,1,1,1,1,0,0);
    rotD_T(500);
    movT(500);

    acendeLeds(0,0,1,1,1,1,1,0);
    rotE_T(500);
    movT(500);

    acendeLeds(0,0,1,1,1,1,0,0);
    rotD_T(500);
    movT(500);

    acendeLeds(0,0,1,1,1,1,1,0);
    rotE_T(500);
    movT(500);

      ////Return to previous point, now going more down////
    acendeLeds(0,0,0,0,1,1,1,1);
    rotD_T(500);
    movT(500);

    acendeLeds(0,0,1,1,1,1,1,0);
    rotE_T(500);
    movT(500);

    acendeLeds(0,0,0,0,1,1,1,1);
    rotD_T(500);
    movT(500);

    acendeLeds(0,0,1,1,1,1,1,0);
    rotE_T(500);
    movT(250);

    acendeLeds(0,0,0,0,1,1,1,1);
    rotD_T(500);
    movT(500);

    ligaLeds();
    rotD_F(500);
    movF(1250);

    //Movimento 2 Abaixo
    acendeLeds(1,0,1,0,1,0,1,0);
    movF(1000);
    
    desligaLeds();
    girD(1000);

    acendeLeds(1,0,1,0,1,0,1,0);
    movF(1000);
    
    desligaLeds();
    girD(500);

    acendeLeds(1,0,1,0,1,0,1,0);;
    movF(1000);
    
    desligaLeds();
    girD(1000);

    acendeLeds(1,0,1,0,1,0,1,0);
    movF(1000);
    
    desligaLeds();
    girE(1000);

    acendeLeds(1,0,1,0,1,0,1,0);
    movT(1000);
    
    desligaLeds();
    girE(1000);

    acendeLeds(1,0,1,0,1,0,1,0);
    movT(1000);
    
    desligaLeds();
    girE(500);

    acendeLeds(1,0,1,0,1,0,1,0);
    movT(1000);
    
    desligaLeds();
    girE(1000);

    acendeLeds(1,0,1,0,1,0,1,0);
    movT(1000);

    ligaLeds();
    girD(3000);

    //Movimento 3 abaixo
    desligaLeds();
    movF(1250);

    acendeLeds(1,0,0,0,1,1,1,1);
    rotE_F(1500);

    desligaLeds();
    movF(1750);

    acendeLeds(1,1,1,1,1,0,0,0);
    rotD_F(1500);

    desligaLeds();
    movF(1250);

    acendeLeds(1,0,0,0,1,1,1,1);
    rotE_F(1500);

    desligaLeds();
    movF(1750);

    acendeLeds(1,1,1,1,1,0,0,0);
    rotD_F(1500);

    desligaLeds();
    movF(750);

    acendeLeds(0,1,1,1,0,1,1,1);
    girD(2000);

    //Movimento 4 Abaixo
    acendeLeds(1,0,0,1,0,1,0,0);
    para();
    delay(500);
    girD(4000);

    acendeLeds(0,1,0,0,1,0,0,1);
    para();
    delay(1000);
    girE(4000);

    acendeLeds(1,0,1,1,1,0,1,1);
    para();
    delay(1000);
    girD(1000);

    acendeLeds(1,1,1,0,1,1,1,0);
    para();
    delay(500);
    girE(1000);

    acendeLeds(1,1,0,0,0,0,0,0);
    para();
    delay(500);
    girE(500);

    acendeLeds(1,1,1,1,0,0,0,0);
    para();
    delay(250);
    girE(500);

    acendeLeds(1,1,1,1,1,1,0,0);
    para();
    delay(250);
    girE(500);

    ligaLeds();
    para();
    delay(250);
    girE(500);
    para();
    delay(1250);

    acendeLeds(0,0,1,1,1,1,1,1);
    girD(500);
    para();
    delay(250);

    acendeLeds(0,0,0,0,1,1,1,1);
    girD(500);
    para();
    delay(250);

    acendeLeds(0,0,0,0,0,0,1,1);
    girD(500);
    para();
    delay(250);

    desligaLeds();
    girD(500);
    para();
    delay(250);
    girD(2000);

    ligaLeds();
    girE(2000);

    //Movimento 5 abaixo
    desligaLeds();
    girD(500);

    acendeLeds(0,0,0,0,0,0,0,1);
    rotE_F(1500);

    acendeLeds(0,0,0,0,0,0,1,1);
    rotD_F(1500);

    acendeLeds(0,0,0,0,0,1,1,1);
    rotE_F(1500);

    acendeLeds(0,0,0,0,1,1,1,1);
    rotD_F(1500);

    acendeLeds(1,1,1,1,0,0,0,0);
    girE(1000);

    acendeLeds(1,1,1,1,0,0,0,0);
    rotE_T(1500);

    acendeLeds(1,1,1,1,1,0,0,0);
    rotD_T(1500);

    acendeLeds(1,1,1,1,1,1,0,0);
    rotE_T(1500);

    acendeLeds(1,1,1,1,1,1,1,0);
    rotD_T(1500);

    ligaLeds();
    girD(1000);

    acendeLeds(0,0,0,0,1,1,1,1);
    rotE_T(1500);

    acendeLeds(1,1,1,1,0,0,0,0);
    rotD_F(1500);

    ligaLeds();
    movF(2000);

    acendeLeds(0,1,1,1,1,0,0,0);
    girE(250);

    acendeLeds(1,1,1,0,0,0,1,1);
    movF(2750);

    desligaLeds();
    girE(2250);

    //Movimento 6 Abaixo
    acendeLeds(1,1,1,0,0,1,0,0);
    rotD_F(3000);

    acendeLeds(1,0,0,1,0,0,1,1);
    rotE_F(3000);

    acendeLeds(0,0,1,1,1,0,0,1);
    rotD_T(3000);

    acendeLeds(0,1,0,0,1,1,1,0);
    rotE_T(3000);

    acendeLeds(1,0,1,0,1,0,1,0);
    girE(2000);

    acendeLeds(0,1,0,1,0,1,0,1);
    girD(2000);

    acendeLeds(1,0,0,0,1,0,0,0);
    girE(1000);

    acendeLeds(0,0,0,1,0,0,0,1);
    girD(1000);

    acendeLeds(0,0,1,0,0,0,1,0);
    girE(1000);

    acendeLeds(0,1,0,0,0,1,0,0);
    girD(1000);

    //Movimento 7 Abaixo
    desligaLeds();
    girE(250);

    acendeLeds(1,1,1,1,0,0,0,0);
    rotE_F(1500);

    acendeLeds(0,0,0,0,1,1,1,1);
    rotD_T(1500);

    acendeLeds(0,0,1,1,1,1,0,0);
    rotE_F(1500);

    ligaLeds();
    girD(750);

    acendeLeds(1,1,0,1,1,1,0,1);
    movF(1500);
    girE(1000);
    movT(1000);

    desligaLeds();
    girD(500);

    acendeLeds(1,1,0,1,1,1,0,1);
    movT(1500);
    girD(1000);
    movF(500);

    //Movimento 8 abaixo
    desligaLeds();
    rotD_F(250);
    movF(500);

    acendeLeds(1,0,0,0,0,0,0,0);
    rotD_F(250);

    acendeLeds(1,1,0,0,0,0,0,0);
    movF(500);

    acendeLeds(1,1,1,0,0,0,0,0);
    rotD_F(250);

    acendeLeds(1,1,1,1,0,0,0,0);
    movF(500);

    acendeLeds(1,1,1,1,1,0,0,0);
    rotD_F(250);

    acendeLeds(1,1,1,1,1,1,0,0);
    movF(500);

    acendeLeds(1,1,1,1,1,1,1,0);
    rotD_F(250);

    desligaLeds();
    movF(500);

    acendeLeds(1,0,0,0,0,0,0,0);
    rotD_F(250);

    acendeLeds(1,1,0,0,0,0,0,0);
    movF(500);

    acendeLeds(1,1,1,0,0,0,0,0);
    rotD_F(250);

    acendeLeds(1,1,1,1,0,0,0,0);
    movF(500);

    acendeLeds(1,1,1,1,1,0,0,0);
    rotD_F(250);

    acendeLeds(1,1,1,1,1,1,0,0);
    movF(500);

    acendeLeds(1,1,1,1,1,1,1,0);
    rotD_F(250);

    acendeLeds(1,0,0,0,0,0,0,0);
    movF(500);

    acendeLeds(1,1,0,0,0,0,0,0);
    rotD_F(250);

    acendeLeds(1,1,1,0,0,0,0,0);
    movF(250);

    acendeLeds(1,1,1,1,0,0,0,0);
    rotD_F(250);

    acendeLeds(1,1,1,1,1,0,0,0);
    movF(250);

    acendeLeds(1,1,1,1,1,1,0,0);
    rotD_F(250);

    acendeLeds(1,1,1,1,1,1,1,0);
    movF(250);

    ligaLeds();
    girE(2000);

    //Completou uma volta

    acendeLeds(0,1,1,1,1,1,1,1);
    rotD_T(250);
    movT(500);
    rotD_T(250);

    acendeLeds(0,0,1,1,1,1,1,1);
    movT(500);
    rotD_T(250);
    movT(500);

    acendeLeds(0,0,0,1,1,1,1,1);
    rotD_T(250);
    movT(500);
    rotD_T(250);

    acendeLeds(0,0,0,0,1,1,1,1);
    movT(500);
    rotD_T(250);
    movT(500);

    acendeLeds(0,0,0,0,0,1,1,1);
    rotD_T(250);
    movT(500);
    rotD_T(250);

    acendeLeds(0,0,0,0,0,0,1,1);
    movT(500);
    rotD_T(250);
    movT(500);

    acendeLeds(0,0,0,0,0,0,0,1);
    rotD_T(250);
    movT(500);
    rotD_T(250);

    desligaLeds();
    movT(500);
    rotD_T(250);
    movT(500);
    
    //Movimento 9 abaixo
    acendeLeds(1,1,1,0,0,0,0,0);
    rotD_T(750);
    movT(500);

    acendeLeds(0,0,0,0,1,1,1,0);
    rotE_T(750);
    movT(500);

    acendeLeds(1,1,1,0,0,0,0,0);
    rotD_T(750);

    desligaLeds();
    rotE_T(750);

    //Movimento 10 abaixo
    girD(250);

    acendeLeds(1,1,1,0,0,0,1,1);
    movF(2000);

    desligaLeds();
    girE(750);

    acendeLeds(1,1,1,1,0,1,1,1);
    movF(3000);

    desligaLeds();
    girD(1250);

    acendeLeds(1,1,0,0,0,0,0,1);
    movF(2000);

    desligaLeds();
    girE(500);

    acendeLeds(0,0,0,1,1,1,0,0);
    movT(2000);

    desligaLeds();
    girD(1250);

    acendeLeds(0,1,1,1,1,1,1,1);
    movT(3000);

    desligaLeds();
    girE(750);

    acendeLeds(0,0,1,1,1,1,1,0);
    movT(2000);

    ligaLeds();
    girE(3250);
    //Fim da Coreografia

    break;
    
  }
  tempo = 1;
}
