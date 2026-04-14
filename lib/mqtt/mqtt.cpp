#include "mqtt.h"
#include "diff_car.h"
#include <QuickPID.h>
#include "network_config.h"
#include <Preferences.h>
#include <Arduino.h>
#include <cstring>
#define DEBUG_MQTT

extern QuickPID left_pid;
extern QuickPID right_pid;

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
      client.subscribe("/parametros");
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

  if (String(topic) == "/parametros") {
      // Formato esperado: Ke:1.0 Kde:1.0 Kout:1.0 MaxInc:0.025
      float Kp_left, Ki_left, Kd_left, Kp_right, Ki_right, Kd_right;

      int idx;
      if ((idx = mensagem.indexOf("Kp_left:")) >= 0) Kp_left = mensagem.substring(idx + 8).toFloat();
      if ((idx = mensagem.indexOf("Ki_left:")) >= 0) Kd_left = mensagem.substring(idx + 8).toFloat();
      if ((idx = mensagem.indexOf("Kd_left:")) >= 0) Kp_right = mensagem.substring(idx + 9).toFloat();
      if ((idx = mensagem.indexOf("Kp_right:")) >= 0) Kp_left = mensagem.substring(idx + 9).toFloat();
      if ((idx = mensagem.indexOf("Ki_right:")) >= 0) Kp_right = mensagem.substring(idx + 9).toFloat();
      if ((idx = mensagem.indexOf("Kd_right:")) >= 0) Kd_right = mensagem.substring(idx + 9).toFloat();

      Preferences prefs;
      prefs.begin("fuzzy", false);
      prefs.putFloat("Kp_left", Kp_left);
      prefs.putFloat("Ki_left", Ki_left);
      prefs.putFloat("Kd_left", Kd_left);
      prefs.putFloat("Kp_right", Kp_right);
      prefs.putFloat("Ki_right", Ki_right);
      prefs.putFloat("Kd_right", Kd_right);
      prefs.end();

      left_pid.SetTunings(Kp_left, Ki_left, Kd_left);
      right_pid.SetTunings(Kp_right, Ki_right, Kd_right);
      Serial.printf("Parametros Fuzzy Atualizados: Kp_left=%.3f Kd_left=%.3f Kp_right=%.3f Kd_right=%.3f\n", Kp_left, Kd_left, Kp_right, Kd_right);
      return;
  }

  // Defensive checks before indexing
  if (mensagem.length() >= 3 && mensagem[0] == 'D' && mensagem[1] == 'N' && mensagem[2] == 'X') {
    if (mensagem.length() >= 6) {
      if (mensagem[4] == 'C' && mensagem[5] == 'G') {
        Serial.println("Iniciando coreografia!");
        coreo();
      } else if (mensagem[4] == 'M' && mensagem[5] == 'F') {
        Serial.println("Movendo para frente!");
        movF_Reto(100);
      } else if (mensagem[4] == 'M' && mensagem[5] == 'T') {
        movT_Reto(100);
      } else if (mensagem[4] == 'M' && mensagem[5] == 'D') {
        MovDistancia(100, 0.3f);
      } else if (mensagem[4] == 'M' && mensagem[5] == 'E') {
        MovDistancia(-100, 0.3f);
      } else if (mensagem[4] == 'G' && mensagem[5] == 'D') {
        girD(1);
      } else if (mensagem[4] == 'G' && mensagem[5] == 'E') {
        girE(1);
      } else if (mensagem[4] == 'P' && mensagem[5] == 'A') {
        para();
      } else if (mensagem[4] == 'N' && mensagem[5] == 'V') {
        diffCar.mode = 1;
        double dist = mensagem.substring(7).toDouble();
        if (mensagem[6] == '-'){
          dist = -dist;
        }
        diffCar.mover_distancia(dist, 0.4f);
      } else if (mensagem[4] == 'G' && mensagem[5] == 'O') {
        int idx1 = mensagem.indexOf(':', 7);
        int idx2 = mensagem.indexOf(':', idx1 + 1);
        if (idx1 > 0 && idx2 > 0) {
           diffCar.target_x = mensagem.substring(7, idx1).toFloat();
           diffCar.target_y = mensagem.substring(idx1 + 1, idx2).toFloat();
           diffCar.target_theta = mensagem.substring(idx2 + 1).toFloat();
           diffCar.mode = 1; 
        }
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
  telemetry += "\"left_velocity_cms\":" + String(diffCar.left_velocity_cms) + ",";
  telemetry += "\"right_velocity_cms\":" + String(diffCar.right_velocity_cms) + ",";
  telemetry += "\"left_vel_target\":" + String(diffCar.left_velocity_target) + ",";
  telemetry += "\"right_vel_target\":" + String(diffCar.right_velocity_target) + ",";
  telemetry += "\"x\":" + String(diffCar.odom_real.x/100.0) + ",";
  telemetry += "\"y\":" + String(diffCar.odom_real.y/100.0) + ",";
  telemetry += "\"theta\":" + String(diffCar.odom_real.theta) + ",";
  telemetry += "\"accel_x\":" + String(diffCar.accel_d[0]) + ",";
  telemetry += "\"accel_y\":" + String(diffCar.accel_d[1]) + ",";
  telemetry += "\"accel_z\":" + String(diffCar.accel_d[2]) + ",";
  telemetry += "\"gyro_x\":" + String(diffCar.gyro_d[0]) + ",";
  telemetry += "\"gyro_y\":" + String(diffCar.gyro_d[1]) + ",";
  telemetry += "\"gyro_z\":" + String(diffCar.gyro_d[2]) + ",";
  telemetry += "\"yaw_imu\":" + String(diffCar.ypr_d[0]);
  telemetry += "}";
  publish("telemetry", telemetry.c_str());

  static unsigned long last_param_pub = 0;
  if (millis() - last_param_pub >= 3000) {
    last_param_pub = millis();
    String params = "Kp:" + String(left_pid.GetKp()) + 
                    " Ki:" + String(left_pid.GetKi()) + 
                    " Kd:" + String(left_pid.GetKd());
    publish("/parametros/atuais", params.c_str());
  }
}

