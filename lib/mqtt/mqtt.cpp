#include "mqtt.h"
#include "diff_car.h"
#include "network_config.h"
#include <Arduino.h>
#include <cstring>
#define COMUM_SPEED 0.2f // Velocidade comum para seguir linha

#define DEBUG_MQTT

WiFiClient espClient;
PubSubClient client(espClient);
IPAddress mqtt_server(10, 0, 0, 134);

namespace {
constexpr unsigned long WIFI_RETRY_INTERVAL_MS = WIFI_RETRY_DELAY_MS;
constexpr unsigned long WIFI_CONNECT_WAIT_STEP_MS = 500;
unsigned long lastWifiAttemptMs = 0;
bool wifiBeginIssued = false;
bool wifiWarnedMissingCredentials = false;
}

void MqttTask::setup() {
  waitForWifi();
  client.setServer(mqtt_server, 1883); 
  client.setCallback([this](char* topic, byte* payload, unsigned int length) {
    this->callback(topic, payload, length);
  });
}

bool MqttTask::reconnect() {
  if (!ensureWifiConnected()) {
    return false;
  }
  // Loop repetido até o estabelecimento da reconexão ->
  if (!client.connected()) {
    MQTT_Connected = false;
    Serial.print("Attempting MQTT connection...");

    // Criação de um "Client ID" randômico:
    String clientId = "dan1";
    clientId += String(random(0xffff), HEX);

    // Tentativa de conexão:
    if(client.connect(clientId.c_str())) {
      MQTT_Connected = true;
      Serial.println("connected");
      client.subscribe("cmd");
    }else{
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
    }
  }
  return client.connected();
}

void MqttTask::callback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived on topic: ");
    Serial.print(topic);
    Serial.print(". Message: ");
    String messageTemp;
    
    for (int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
      messageTemp += (char)payload[i];
    }
    Serial.println();

  // Build a safe String from payload (messageTemp already contains it)
  messageTemp.trim(); // remove trailing newline/carriage returns
  String mensagem = messageTemp;

  // Defensive checks before indexing
  if (mensagem.length() >= 3 && mensagem[0] == 'D' && mensagem[1] == 'N' && mensagem[2] == 'X') {
    if (mensagem.length() >= 6) {
      if (mensagem[4] == 'C' && mensagem[5] == 'G') {
        Serial.println("Iniciando coreografia!");
        coreo();
      } else if (mensagem[4] == 'M' && mensagem[5] == 'F') {
        Serial.println("Movendo para frente!");
        movF(1);
      } else if (mensagem[4] == 'M' && mensagem[5] == 'T') {
        movT(1);
      } else if (mensagem[4] == 'G' && mensagem[5] == 'D') {
        girD(1);
      } else if (mensagem[4] == 'G' && mensagem[5] == 'E') {
        girE(1);
      } else if (mensagem[4] == 'P' && mensagem[5] == 'A') {
        para();
      } else {
        Serial.print("Unknown DNX command: ");
        Serial.println(mensagem.substring(4));
      }
    } else {
      Serial.println("DNX message too short");
    }
  }
}

void MqttTask::handler() {
  if (!ensureWifiConnected()) {
    return;
  }
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}

bool MqttTask::ensureWifiConnected() {
  if (strlen(WIFI_SSID) == 0 && !wifiWarnedMissingCredentials) {
    Serial.println("[WiFi] WIFI_SSID not configured. Define WIFI_SSID/WIFI_PASSWORD in network_config.h or via build_flags.");
    wifiWarnedMissingCredentials = true;
    return false;
  }

  if (WiFi.status() == WL_CONNECTED) {
    return true;
  }

  const unsigned long now = millis();
  if (!wifiBeginIssued || now - lastWifiAttemptMs >= WIFI_RETRY_INTERVAL_MS) {
    Serial.print("[WiFi] Connecting to SSID: ");
    Serial.println(WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    wifiBeginIssued = true;
    lastWifiAttemptMs = now;
  }
  return false;
}

void MqttTask::waitForWifi() {
  const unsigned long start = millis();
  while (millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
    if (ensureWifiConnected()) {
      Serial.print("[WiFi] Connected, IP: ");
      Serial.println(WiFi.localIP());
      return;
    }
    delay(WIFI_CONNECT_WAIT_STEP_MS);
  }
  Serial.println("[WiFi] Connection timeout. MQTT will retry in background.");
}

void MqttTask::publish(const std::string& topic, const std::string& message) {
  if (client.connected()) {
    client.publish(topic.c_str(), message.c_str());
  }
}

void MqttTask::updateTelemetry() {
  if (!client.connected()) {
    return;
  }
  // Exemplo de publicação de telemetria
  String telemetry = "{";
  telemetry += "\"left_velocity_ms\":" + String(diffCar.left_velocity_ms) + ",";
  telemetry += "\"right_velocity_ms\":" + String(diffCar.right_velocity_ms) + ",";
  telemetry += "}";
  publish("telemetry", telemetry.c_str());
}

void MqttTask::girE(int tempoGiro){
  diffCar.left_motor_dir = false;
  diffCar.right_motor_dir = true;
  diffCar.left_velocity_target = COMUM_SPEED;
  diffCar.right_velocity_target = COMUM_SPEED;
}

void MqttTask::girD(int tempoGiro){
  diffCar.left_motor_dir = true;
  diffCar.right_motor_dir = false;
  diffCar.left_velocity_target = COMUM_SPEED;
  diffCar.right_velocity_target = COMUM_SPEED;
}

void MqttTask::para(){
  diffCar.left_velocity_target = 0.0f;
  diffCar.right_velocity_target = 0.0f;
}

void MqttTask::movF(int tempoMov){
  diffCar.left_motor_dir = true;
  diffCar.right_motor_dir = true;
  diffCar.left_velocity_target = COMUM_SPEED;
  diffCar.right_velocity_target = COMUM_SPEED;
}

void MqttTask::movT(int tempoMov){
  diffCar.left_motor_dir = false;
  diffCar.right_motor_dir = false;
  diffCar.left_velocity_target = COMUM_SPEED;
  diffCar.right_velocity_target = COMUM_SPEED;
}

void MqttTask::rotD_F(int tempoMov) {
  diffCar.left_motor_dir = false;
  diffCar.right_motor_dir = false;
  diffCar.left_velocity_target = 0;
  diffCar.right_velocity_target = COMUM_SPEED;
}

void MqttTask::rotE_F(int tempoMov) {
  diffCar.left_motor_dir = false;
  diffCar.right_motor_dir = false;
  diffCar.left_velocity_target = COMUM_SPEED;
  diffCar.right_velocity_target = 0;
}

void MqttTask::rotD_T(int tempoMov) {
  diffCar.left_motor_dir = true;
  diffCar.right_motor_dir = true;
  diffCar.left_velocity_target = 0;
  diffCar.right_velocity_target = COMUM_SPEED;
}

void MqttTask::rotE_T(int tempoMov) {
  diffCar.left_motor_dir = true;
  diffCar.right_motor_dir = true;
  diffCar.left_velocity_target = COMUM_SPEED;
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
